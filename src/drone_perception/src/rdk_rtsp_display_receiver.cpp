#include <getopt.h>
#include <signal.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <sp_codec.h>
#include <sp_display.h>
#include <sp_sys.h>
#include <sp_vio.h>
}

namespace
{

volatile sig_atomic_t g_stop_requested = 0;

void handleSignal(int)
{
  g_stop_requested = 1;
}

struct Options
{
  std::string input;
  std::string transport{"udp"};
};

void printUsage(const char * program)
{
  std::fprintf(
    stderr,
    "Usage: %s --input RTSP_URL [--transport udp|tcp]\n"
    "  --input URL       MediaMTX H.264 stream URL\n"
    "  --transport TYPE  RTP transport (default: udp)\n"
    "  --help            Show this message\n",
    program);
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  const option long_options[] = {
    {"input", required_argument, nullptr, 'i'},
    {"transport", required_argument, nullptr, 't'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, 0, nullptr, 0},
  };

  while (true) {
    const int key = getopt_long(argc, argv, "i:t:h", long_options, nullptr);
    if (key == -1) {
      break;
    }
    switch (key) {
      case 'i': options.input = optarg; break;
      case 't': options.transport = optarg; break;
      case 'h':
        printUsage(argv[0]);
        std::exit(0);
      default:
        printUsage(argv[0]);
        std::exit(2);
    }
  }

  if (options.input.empty()) {
    throw std::invalid_argument("--input is required");
  }
  if (options.transport != "udp" && options.transport != "tcp") {
    throw std::invalid_argument("--transport must be udp or tcp");
  }
  return options;
}

std::string avError(int result)
{
  char buffer[AV_ERROR_MAX_STRING_SIZE]{};
  av_strerror(result, buffer, sizeof(buffer));
  return buffer;
}

class RdkDisplayPipeline
{
public:
  RdkDisplayPipeline(
    int stream_width, int stream_height,
    const uint8_t * decoder_extradata, int decoder_extradata_size)
  {
    try {
      decoder_ = sp_init_decoder_module();
      display_ = sp_init_display_module();
      vps_ = sp_init_vio_module();
      if (decoder_ == nullptr || display_ == nullptr || vps_ == nullptr) {
        throw std::runtime_error("failed to initialize an RDK media module");
      }

      int display_widths[20]{};
      int display_heights[20]{};
      sp_get_display_resolution(display_widths, display_heights);
      selectDisplayResolution(
        stream_width, stream_height, display_widths, display_heights,
        display_width_, display_height_);
      if (display_width_ <= 0 || display_height_ <= 0) {
        throw std::runtime_error(
                "no HDMI mode is compatible with the RTSP stream resolution");
      }

      int result = sp_start_decode(
        decoder_, "", 0, SP_ENCODER_H264, stream_width, stream_height);
      if (result != 0) {
        throwRdkError("sp_start_decode", result);
      }
      decoder_started_ = true;

      if (decoder_extradata != nullptr && decoder_extradata_size > 0) {
        result = sp_decoder_set_image(
          decoder_, const_cast<char *>(
            reinterpret_cast<const char *>(decoder_extradata)),
          0, decoder_extradata_size, 0);
        if (result != 0) {
          throwRdkError("sp_decoder_set_image(extradata)", result);
        }
        std::fprintf(stderr, "sent %d bytes of H.264 SPS/PPS\n", decoder_extradata_size);
      }

      result = sp_start_display(display_, 1, display_width_, display_height_);
      if (result != 0) {
        throwRdkError("sp_start_display", result);
      }
      display_started_ = true;

      result = sp_open_vps(
        vps_, 0, 1, SP_VPS_SCALE, stream_width, stream_height,
        &display_width_, &display_height_, nullptr, nullptr, nullptr, nullptr, nullptr);
      if (result != 0) {
        throwRdkError("sp_open_vps", result);
      }
      vps_started_ = true;

      result = sp_module_bind(decoder_, SP_MTYPE_DECODER, vps_, SP_MTYPE_VIO);
      if (result != 0) {
        throwRdkError("bind decoder to VPS", result);
      }
      decoder_bound_ = true;

      result = sp_module_bind(vps_, SP_MTYPE_VIO, display_, SP_MTYPE_DISPLAY);
      if (result != 0) {
        throwRdkError("bind VPS to display", result);
      }
      display_bound_ = true;

      std::fprintf(
        stderr, "RDK display ready: stream %dx%d -> HDMI %dx%d\n",
        stream_width, stream_height, display_width_, display_height_);
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~RdkDisplayPipeline()
  {
    cleanup();
  }

  RdkDisplayPipeline(const RdkDisplayPipeline &) = delete;
  RdkDisplayPipeline & operator=(const RdkDisplayPipeline &) = delete;

  void send(const uint8_t * data, int size)
  {
    const int result = sp_decoder_set_image(
      decoder_, const_cast<char *>(reinterpret_cast<const char *>(data)), 0, size, 0);
    if (result != 0) {
      throwRdkError("sp_decoder_set_image", result);
    }
  }

private:
  static void selectDisplayResolution(
    int stream_width, int stream_height,
    const int * widths, const int * heights,
    int & selected_width, int & selected_height)
  {
    selected_width = 0;
    selected_height = 0;
    for (int index = 0; index < 20 && widths[index] > 0; ++index) {
      if (stream_width >= widths[index] && stream_height >= heights[index]) {
        selected_width = widths[index];
        selected_height = heights[index];
        break;
      }
    }
  }

  [[noreturn]] static void throwRdkError(const char * operation, int result)
  {
    throw std::runtime_error(
            std::string(operation) + " failed, result=" + std::to_string(result));
  }

  void cleanup()
  {
    if (display_bound_) {
      sp_module_unbind(vps_, SP_MTYPE_VIO, display_, SP_MTYPE_DISPLAY);
      display_bound_ = false;
    }
    if (decoder_bound_) {
      sp_module_unbind(decoder_, SP_MTYPE_DECODER, vps_, SP_MTYPE_VIO);
      decoder_bound_ = false;
    }
    if (display_started_) {
      sp_stop_display(display_);
      display_started_ = false;
    }
    if (display_ != nullptr) {
      sp_release_display_module(display_);
      display_ = nullptr;
    }
    if (vps_started_) {
      sp_vio_close(vps_);
      vps_started_ = false;
    }
    if (vps_ != nullptr) {
      sp_release_vio_module(vps_);
      vps_ = nullptr;
    }
    if (decoder_started_) {
      sp_stop_decode(decoder_);
      decoder_started_ = false;
    }
    if (decoder_ != nullptr) {
      sp_release_decoder_module(decoder_);
      decoder_ = nullptr;
    }
  }

  void * decoder_{nullptr};
  void * display_{nullptr};
  void * vps_{nullptr};
  int display_width_{0};
  int display_height_{0};
  bool decoder_started_{false};
  bool display_started_{false};
  bool vps_started_{false};
  bool decoder_bound_{false};
  bool display_bound_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  AVFormatContext * format_context = nullptr;
  AVPacket * packet = nullptr;
  AVDictionary * dictionary = nullptr;

  try {
    const Options options = parseOptions(argc, argv);
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);
    avformat_network_init();

    av_dict_set(&dictionary, "stimeout", "3000000", 0);
    av_dict_set(&dictionary, "buffer_size", "1024000", 0);
    av_dict_set(&dictionary, "rtsp_transport", options.transport.c_str(), 0);

    int result = avformat_open_input(
      &format_context, options.input.c_str(), nullptr, &dictionary);
    av_dict_free(&dictionary);
    if (result < 0) {
      throw std::runtime_error("cannot open RTSP input: " + avError(result));
    }
    result = avformat_find_stream_info(format_context, nullptr);
    if (result < 0) {
      throw std::runtime_error("cannot read RTSP stream information: " + avError(result));
    }

    const int video_index = av_find_best_stream(
      format_context, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (video_index < 0) {
      throw std::runtime_error("RTSP input has no video stream: " + avError(video_index));
    }
    AVCodecParameters * codec = format_context->streams[video_index]->codecpar;
    if (codec->codec_id != AV_CODEC_ID_H264) {
      throw std::runtime_error("RTSP video is not H.264");
    }
    if (codec->width <= 0 || codec->height <= 0) {
      throw std::runtime_error("RTSP video has invalid dimensions");
    }

    std::fprintf(
      stderr, "Receiving %s over RTP/%s, H.264 %dx%d\n",
      options.input.c_str(), options.transport.c_str(), codec->width, codec->height);
    RdkDisplayPipeline pipeline(
      codec->width, codec->height, codec->extradata, codec->extradata_size);

    packet = av_packet_alloc();
    if (packet == nullptr) {
      throw std::runtime_error("av_packet_alloc failed");
    }

    while (!g_stop_requested) {
      result = av_read_frame(format_context, packet);
      if (result == AVERROR(EAGAIN)) {
        continue;
      }
      if (result < 0) {
        throw std::runtime_error("RTSP read failed: " + avError(result));
      }
      if (packet->stream_index == video_index) {
        pipeline.send(packet->data, packet->size);
      }
      av_packet_unref(packet);
    }

    av_packet_free(&packet);
    avformat_close_input(&format_context);
    avformat_network_deinit();
    return 0;
  } catch (const std::exception & error) {
    av_dict_free(&dictionary);
    av_packet_free(&packet);
    if (format_context != nullptr) {
      avformat_close_input(&format_context);
    }
    avformat_network_deinit();
    std::fprintf(stderr, "rdk_rtsp_display_receiver: %s\n", error.what());
    return 1;
  }
}

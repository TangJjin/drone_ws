#include <fcntl.h>
#include <getopt.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <hb_media_codec.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
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
  std::string device{"/dev/d435i_color"};
  int width{640};
  int height{480};
  int fps{30};
  int bitrate_kbps{5000};
  int gop{15};
  int camera_timeout_ms{3000};
};

void printUsage(const char * program)
{
  std::fprintf(
    stderr,
    "Usage: %s [options] > stream.h264\n"
    "  --device PATH           V4L2 RGB device (default: /dev/d435i_color)\n"
    "  --width PIXELS          Width (default: 640)\n"
    "  --height PIXELS         Height (default: 480)\n"
    "  --fps FPS               Frame rate (default: 30)\n"
    "  --bitrate-kbps RATE     H.264 CBR rate (default: 5000)\n"
    "  --gop FRAMES            IDR interval (default: 15)\n"
    "  --camera-timeout-ms MS  V4L2 poll timeout (default: 3000)\n"
    "  --help                  Show this message\n",
    program);
}

int parsePositive(const char * value, const char * name)
{
  char * end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed <= 0 || parsed > INT32_MAX) {
    throw std::invalid_argument(std::string("invalid ") + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  const option long_options[] = {
    {"device", required_argument, nullptr, 'd'},
    {"width", required_argument, nullptr, 'w'},
    {"height", required_argument, nullptr, 'h'},
    {"fps", required_argument, nullptr, 'f'},
    {"bitrate-kbps", required_argument, nullptr, 'b'},
    {"gop", required_argument, nullptr, 'g'},
    {"camera-timeout-ms", required_argument, nullptr, 't'},
    {"help", no_argument, nullptr, '?'},
    {nullptr, 0, nullptr, 0},
  };

  while (true) {
    const int key = getopt_long(argc, argv, "", long_options, nullptr);
    if (key == -1) {
      break;
    }
    switch (key) {
      case 'd': options.device = optarg; break;
      case 'w': options.width = parsePositive(optarg, "width"); break;
      case 'h': options.height = parsePositive(optarg, "height"); break;
      case 'f': options.fps = parsePositive(optarg, "fps"); break;
      case 'b': options.bitrate_kbps = parsePositive(optarg, "bitrate-kbps"); break;
      case 'g': options.gop = parsePositive(optarg, "gop"); break;
      case 't':
        options.camera_timeout_ms = parsePositive(optarg, "camera-timeout-ms");
        break;
      case '?':
      default:
        printUsage(argv[0]);
        std::exit(key == '?' ? 0 : 2);
    }
  }

  if ((options.width & 1) != 0 || (options.height & 1) != 0) {
    throw std::invalid_argument("NV12 requires even width and height");
  }
  return options;
}

int xioctl(int fd, unsigned long request, void * argument)
{
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result < 0 && errno == EINTR);
  return result;
}

class V4l2Camera
{
public:
  struct Frame
  {
    const uint8_t * data{nullptr};
    std::size_t size{0};
    uint32_t index{0};
  };

  explicit V4l2Camera(const Options & options)
  : width_(options.width), height_(options.height), timeout_ms_(options.camera_timeout_ms)
  {
    fd_ = open(options.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throwSystemError("open camera " + options.device);
    }

    v4l2_capability capability{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0) {
      throwSystemError("VIDIOC_QUERYCAP");
    }
    if ((capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
      (capability.capabilities & V4L2_CAP_STREAMING) == 0)
    {
      throw std::runtime_error("camera does not support V4L2 streaming capture");
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(width_);
    format.fmt.pix.height = static_cast<uint32_t>(height_);
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
      throwSystemError("VIDIOC_S_FMT");
    }
    if (format.fmt.pix.width != static_cast<uint32_t>(width_) ||
      format.fmt.pix.height != static_cast<uint32_t>(height_) ||
      format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
    {
      throw std::runtime_error("camera rejected requested YUYV format");
    }
    bytes_per_line_ = format.fmt.pix.bytesperline != 0 ?
      static_cast<int>(format.fmt.pix.bytesperline) : width_ * 2;

    v4l2_streamparm stream_parameters{};
    stream_parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_parameters.parm.capture.timeperframe.numerator = 1;
    stream_parameters.parm.capture.timeperframe.denominator = options.fps;
    if (xioctl(fd_, VIDIOC_S_PARM, &stream_parameters) < 0) {
      throwSystemError("VIDIOC_S_PARM");
    }

    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0) {
      throwSystemError("VIDIOC_REQBUFS");
    }
    if (request.count < 2) {
      throw std::runtime_error("camera returned fewer than two mmap buffers");
    }

    buffers_.resize(request.count);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        throwSystemError("VIDIOC_QUERYBUF");
      }
      void * address = mmap(
        nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
      if (address == MAP_FAILED) {
        throwSystemError("mmap camera buffer");
      }
      buffers_[index] = {address, buffer.length};
      if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        throwSystemError("VIDIOC_QBUF");
      }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throwSystemError("VIDIOC_STREAMON");
    }
    streaming_ = true;
  }

  ~V4l2Camera()
  {
    if (streaming_) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (const Buffer & buffer : buffers_) {
      if (buffer.address != nullptr && buffer.address != MAP_FAILED) {
        munmap(buffer.address, buffer.length);
      }
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  V4l2Camera(const V4l2Camera &) = delete;
  V4l2Camera & operator=(const V4l2Camera &) = delete;

  bool dequeue(Frame & frame)
  {
    pollfd descriptor{fd_, POLLIN, 0};
    const int poll_result = poll(&descriptor, 1, timeout_ms_);
    if (poll_result == 0) {
      throw std::runtime_error("camera frame timeout");
    }
    if (poll_result < 0) {
      if (errno == EINTR) {
        return false;
      }
      throwSystemError("poll camera");
    }

    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN) {
        return false;
      }
      throwSystemError("VIDIOC_DQBUF");
    }
    if (buffer.index >= buffers_.size()) {
      throw std::runtime_error("camera returned invalid buffer index");
    }
    frame.data = static_cast<const uint8_t *>(buffers_[buffer.index].address);
    frame.size = buffer.bytesused;
    frame.index = buffer.index;
    return true;
  }

  void requeue(uint32_t index)
  {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
      throwSystemError("VIDIOC_QBUF");
    }
  }

  int bytesPerLine() const {return bytes_per_line_;}

private:
  struct Buffer
  {
    void * address{nullptr};
    std::size_t length{0};
  };

  [[noreturn]] static void throwSystemError(const std::string & operation)
  {
    throw std::runtime_error(operation + ": " + std::strerror(errno));
  }

  int fd_{-1};
  int width_{0};
  int height_{0};
  int timeout_ms_{0};
  int bytes_per_line_{0};
  bool streaming_{false};
  std::vector<Buffer> buffers_;
};

class H264Encoder
{
public:
  explicit H264Encoder(const Options & options)
  : width_(options.width), height_(options.height)
  {
    std::memset(&context_, 0, sizeof(context_));
    context_.encoder = true;
    context_.codec_id = MEDIA_CODEC_ID_H264;

    mc_video_codec_enc_params_t & parameters = context_.video_enc_params;
    parameters.width = width_;
    parameters.height = height_;
    parameters.pix_fmt = MC_PIXEL_FORMAT_NV12;
    parameters.frame_buf_count = 3;
    parameters.external_frame_buf = false;
    parameters.bitstream_buf_count = 3;
    parameters.bitstream_buf_size =
      static_cast<uint32_t>((width_ * height_ * 3 / 2 + 0x3ff) & ~0x3ff);
    parameters.gop_params.gop_preset_idx = 9;
    parameters.rot_degree = MC_CCW_0;
    parameters.mir_direction = MC_DIRECTION_NONE;
    parameters.frame_cropping_flag = false;
    parameters.enable_user_pts = 1;
    parameters.rc_params.mode = MC_AV_RC_MODE_H264CBR;

    int result = hb_mm_mc_get_rate_control_config(&context_, &parameters.rc_params);
    if (result != 0) {
      throwCodecError("hb_mm_mc_get_rate_control_config", result);
    }
    mc_h264_cbr_params_t & rate = parameters.rc_params.h264_cbr_params;
    rate.intra_period = static_cast<uint32_t>(options.gop);
    rate.intra_qp = 30;
    rate.bit_rate = static_cast<uint32_t>(options.bitrate_kbps);
    rate.frame_rate = static_cast<uint32_t>(options.fps);
    rate.initial_rc_qp = 20;
    rate.vbv_buffer_size = 20;
    rate.mb_level_rc_enalbe = 1;
    rate.min_qp_I = 8;
    rate.max_qp_I = 50;
    rate.min_qp_P = 8;
    rate.max_qp_P = 50;
    rate.min_qp_B = 8;
    rate.max_qp_B = 50;
    rate.hvs_qp_enable = 1;
    rate.hvs_qp_scale = 2;
    rate.max_delta_qp = 10;
    rate.qp_map_enable = 0;
    parameters.h264_enc_config.h264_profile = MC_H264_PROFILE_MP;
    parameters.h264_enc_config.h264_level = MC_H264_LEVEL4;

    result = hb_mm_mc_initialize(&context_);
    if (result != 0) {
      throwCodecError("hb_mm_mc_initialize", result);
    }
    initialized_ = true;
    result = hb_mm_mc_configure(&context_);
    if (result != 0) {
      throwCodecError("hb_mm_mc_configure", result);
    }
    mc_av_codec_startup_params_t startup_parameters{};
    result = hb_mm_mc_start(&context_, &startup_parameters);
    if (result != 0) {
      throwCodecError("hb_mm_mc_start", result);
    }
    started_ = true;
  }

  ~H264Encoder()
  {
    if (started_) {
      hb_mm_mc_pause(&context_);
    }
    if (initialized_) {
      hb_mm_mc_release(&context_);
    }
  }

  H264Encoder(const H264Encoder &) = delete;
  H264Encoder & operator=(const H264Encoder &) = delete;

  media_codec_buffer_t dequeueInput()
  {
    media_codec_buffer_t buffer{};
    const int result = hb_mm_mc_dequeue_input_buffer(&context_, &buffer, 2000);
    if (result != 0) {
      throwCodecError("hb_mm_mc_dequeue_input_buffer", result);
    }
    buffer.type = MC_VIDEO_FRAME_BUFFER;
    buffer.vframe_buf.width = width_;
    buffer.vframe_buf.height = height_;
    buffer.vframe_buf.pix_fmt = MC_PIXEL_FORMAT_NV12;
    buffer.vframe_buf.size = static_cast<uint32_t>(width_ * height_ * 3 / 2);
    return buffer;
  }

  void queueInput(media_codec_buffer_t & buffer, uint64_t pts)
  {
    buffer.vframe_buf.pts = pts;
    const int result = hb_mm_mc_queue_input_buffer(&context_, &buffer, 2000);
    if (result != 0) {
      throwCodecError("hb_mm_mc_queue_input_buffer", result);
    }
  }

  media_codec_buffer_t dequeueOutput()
  {
    media_codec_buffer_t buffer{};
    media_codec_output_buffer_info_t information{};
    const int result = hb_mm_mc_dequeue_output_buffer(&context_, &buffer, &information, 2000);
    if (result != 0) {
      throwCodecError("hb_mm_mc_dequeue_output_buffer", result);
    }
    return buffer;
  }

  void releaseOutput(media_codec_buffer_t & buffer)
  {
    const int result = hb_mm_mc_queue_output_buffer(&context_, &buffer, 2000);
    if (result != 0) {
      throwCodecError("hb_mm_mc_queue_output_buffer", result);
    }
  }

private:
  [[noreturn]] static void throwCodecError(const char * operation, int result)
  {
    throw std::runtime_error(
            std::string(operation) + " failed, result=" + std::to_string(result));
  }

  int width_{0};
  int height_{0};
  bool initialized_{false};
  bool started_{false};
  media_codec_context_t context_{};
};

bool writeAll(int fd, const uint8_t * data, std::size_t size)
{
  while (size > 0) {
    const ssize_t written = write(fd, data, size);
    if (written > 0) {
      data += written;
      size -= static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);
    signal(SIGPIPE, SIG_IGN);

    std::fprintf(
      stderr, "D435i sender: %s, %dx%d@%d, H.264 CBR %d kbit/s, GOP %d\n",
      options.device.c_str(), options.width, options.height, options.fps,
      options.bitrate_kbps, options.gop);

    V4l2Camera camera(options);
    H264Encoder encoder(options);
    SwsContext * converter = sws_getContext(
      options.width, options.height, AV_PIX_FMT_YUYV422,
      options.width, options.height, AV_PIX_FMT_NV12,
      SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    if (converter == nullptr) {
      throw std::runtime_error("sws_getContext failed");
    }

    uint64_t frame_number = 0;
    uint64_t discarded_frames = 0;
    while (!g_stop_requested) {
      V4l2Camera::Frame camera_frame;
      if (!camera.dequeue(camera_frame)) {
        continue;
      }

      const std::size_t minimum_size =
        static_cast<std::size_t>(camera.bytesPerLine()) * options.height;
      if (camera_frame.size < minimum_size) {
        ++discarded_frames;
        camera.requeue(camera_frame.index);
        continue;
      }

      media_codec_buffer_t input = encoder.dequeueInput();
      const uint8_t * source_data[4] = {camera_frame.data, nullptr, nullptr, nullptr};
      const int source_stride[4] = {camera.bytesPerLine(), 0, 0, 0};
      uint8_t * destination_data[4] = {
        input.vframe_buf.vir_ptr[0], input.vframe_buf.vir_ptr[1], nullptr, nullptr};
      const int destination_stride[4] = {options.width, options.width, 0, 0};
      const int converted = sws_scale(
        converter, source_data, source_stride, 0, options.height,
        destination_data, destination_stride);
      camera.requeue(camera_frame.index);
      if (converted != options.height) {
        sws_freeContext(converter);
        throw std::runtime_error("sws_scale returned an incomplete frame");
      }

      encoder.queueInput(input, frame_number);
      media_codec_buffer_t output = encoder.dequeueOutput();
      const bool output_ok = writeAll(
        STDOUT_FILENO, output.vstream_buf.vir_ptr, output.vstream_buf.size);
      encoder.releaseOutput(output);
      if (!output_ok) {
        sws_freeContext(converter);
        throw std::runtime_error("H.264 output pipe closed");
      }

      ++frame_number;
      if (frame_number % 300 == 0) {
        std::fprintf(
          stderr, "encoded %llu frames, discarded %llu short camera frames\n",
          static_cast<unsigned long long>(frame_number),
          static_cast<unsigned long long>(discarded_frames));
      }
    }

    sws_freeContext(converter);
    std::fprintf(
      stderr, "sender stopped after %llu frames (%llu discarded)\n",
      static_cast<unsigned long long>(frame_number),
      static_cast<unsigned long long>(discarded_frames));
    return 0;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "rdk_d435i_h264_sender: %s\n", error.what());
    return 1;
  }
}

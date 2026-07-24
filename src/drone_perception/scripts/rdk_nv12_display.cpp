#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

struct Options {
  std::string input;
  int width{1280};
  int height{720};
  int visible_height{0};
  std::string window{"RDK Video"};
};

bool read_exact(const int fd, std::vector<uint8_t> &buffer)
{
  size_t offset = 0;
  while (offset < buffer.size()) {
    const ssize_t count = read(fd, buffer.data() + offset, buffer.size() - offset);
    if (count == 0) {
      return false;
    }
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<size_t>(count);
  }
  return true;
}

bool parse_options(const int argc, char **argv, Options &options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--input" && i + 1 < argc) {
      options.input = argv[++i];
    } else if (arg == "--width" && i + 1 < argc) {
      options.width = std::stoi(argv[++i]);
    } else if (arg == "--height" && i + 1 < argc) {
      options.height = std::stoi(argv[++i]);
    } else if (arg == "--visible-height" && i + 1 < argc) {
      options.visible_height = std::stoi(argv[++i]);
    } else if (arg == "--window" && i + 1 < argc) {
      options.window = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: rdk_nv12_display_cpp --input FIFO [--width N] [--height N] [--visible-height N] [--window NAME]\n";
      return false;
    } else {
      std::cerr << "Unknown or incomplete option: " << arg << "\n";
      return false;
    }
  }
  if (options.visible_height <= 0) {
    options.visible_height = options.height;
  }
  if (options.input.empty() || options.width <= 0 || options.height <= 0 ||
    options.visible_height <= 0 || options.visible_height > options.height) {
    std::cerr << "--input, --width and --height are required\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char **argv)
{
  Options options;
  if (!parse_options(argc, argv, options)) {
    return 2;
  }

  const size_t frame_size = static_cast<size_t>(options.width) * options.height * 3U / 2U;
  std::vector<uint8_t> latest;
  std::mutex latest_mutex;
  std::atomic<bool> stop{false};
  std::atomic<bool> reader_done{false};

  std::thread reader([&]() {
    const int fd = open(options.input.c_str(), O_RDONLY);
    if (fd < 0) {
      std::perror("open NV12 input");
      reader_done = true;
      return;
    }

    std::vector<uint8_t> frame(frame_size);
    while (!stop && read_exact(fd, frame)) {
      {
        std::lock_guard<std::mutex> lock(latest_mutex);
        latest.swap(frame);
      }
      frame.resize(frame_size);
    }
    close(fd);
    reader_done = true;
  });

  cv::namedWindow(options.window, cv::WINDOW_NORMAL);
  cv::resizeWindow(options.window, options.width, options.visible_height);

  uint64_t displayed = 0;
  uint64_t fps_window_frames = 0;
  const auto start = std::chrono::steady_clock::now();
  auto fps_window_start = start;
  double display_fps = 0.0;
  while (!stop) {
    std::vector<uint8_t> frame;
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      frame.swap(latest);
    }

    if (frame.empty()) {
      if (reader_done) {
        break;
      }
      cv::waitKey(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    cv::Mat nv12(options.height * 3 / 2, options.width, CV_8UC1, frame.data());
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    if (options.visible_height != options.height) {
      bgr = bgr(cv::Rect(0, 0, options.width, options.visible_height));
    }
    ++displayed;
    ++fps_window_frames;

    const auto now = std::chrono::steady_clock::now();
    const auto fps_elapsed = std::chrono::duration<double>(now - fps_window_start).count();
    if (fps_elapsed >= 0.5) {
      display_fps = static_cast<double>(fps_window_frames) / fps_elapsed;
      fps_window_frames = 0;
      fps_window_start = now;
    }
    cv::putText(
      bgr,
      cv::format("RDK VPU  %.1f FPS  frames=%llu  %dx%d", display_fps,
        static_cast<unsigned long long>(displayed), options.width, options.visible_height),
      cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::imshow(options.window, bgr);

    const int key = cv::waitKey(1) & 0xff;
    if (key == 27 || key == 'q' || key == 'Q') {
      stop = true;
    }
  }

  stop = true;
  reader.join();
  cv::destroyAllWindows();
  std::cout << "displayed_frames=" << displayed << "\n";
  return 0;
}

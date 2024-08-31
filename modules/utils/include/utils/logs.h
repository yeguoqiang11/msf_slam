#ifndef UTILS_LOG_H
#define UTILS_LOG_H
#include <dirent.h>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "glog/logging.h"
#include "spdlog/async.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
namespace dmlog {
struct LogConfig {
    std::string log_name = "logger";
    std::string file_path = "logs/log.txt";
    spdlog::level::level_enum level = spdlog::level::trace;
    std::string format = "[%Y-%m-%d %H:%M:%S.%2f] [%l] [%s:%# %!()] %v";
    std::string roll_type = "by_size";
    unsigned int reserve_count = 10;
    unsigned int roll_size = 1024 * 1024 * 10; // 10 M
    unsigned int rotation_hour = 0;
    unsigned int rotation_minute = 0;

    unsigned int async_thread_pool_size = 1;
};

class Logger {
  public:
    Logger();
    ~Logger();

    int SetConfig(const LogConfig &config);
    void log(const char* file_name_in, int line_in, const char* func_name_in, spdlog::level::level_enum level,
        const std::string& msg);
  private:
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<spdlog::details::thread_pool> thread_pool_;
};

struct GlogConfig {
  std::string log_name = "slam_log";
  std::string file_dir = "./logs/";
  unsigned int log_size = 10;
};

void SignalHandle(const char *data, int size);
class Glogger {
  public:
    Glogger(GlogConfig config = GlogConfig());
    ~Glogger();
};

} // namespace dmlog
#endif // UTILS_LOG_H
#include "utils/logs.h"

namespace dmlog {
Logger::Logger() {
    spdlog::set_error_handler([](const std::string &msg) {
        std::cout << fmt::format("[{}:{} {}()] {}", __FILE__, __LINE__, __FUNCTION__, msg) << std::endl;
    });
}

Logger::~Logger() {
    spdlog::drop_all();
}

int Logger::SetConfig(const LogConfig &config) {
    int ret = 0;
    do {
        try {
            std::shared_ptr<spdlog::sinks::sink> sink =
                std::make_shared<spdlog::sinks::rotating_file_sink_mt>(config.file_path, config.roll_size, config.reserve_count, true);
            std::unique_ptr<spdlog::pattern_formatter> format =
                std::make_unique<spdlog::pattern_formatter>(config.format, spdlog::pattern_time_type::local, spdlog::details::os::default_eol);
            sink->set_formatter(std::move(format));
            thread_pool_ = std::make_shared<spdlog::details::thread_pool>(config.async_thread_pool_size, 1);
            logger_ = std::make_shared<spdlog::async_logger>(config.log_name, std::move(sink), thread_pool_,
                spdlog::async_overflow_policy::block);
            logger_->flush_on(spdlog::level::trace);
            logger_->set_level(config.level);
            spdlog::register_logger(logger_);
            // spdlog::set_default_logger(logger_);
        } catch (const std::exception &ex) {
            ret = -1;
            std::cout << ex.what() << std::endl;
        } catch (...) {
            ret = -1;
            std::cout << "unknow exception" << std::endl;
        }
    } while (0);
    return ret;
}

void Logger::log(const char* file_name_in, int line_in, const char* func_name_in, spdlog::level::level_enum level,
                 const std::string &msg) {
    try {
        logger_->log(spdlog::source_loc{file_name_in, line_in, func_name_in}, level, msg);
    } catch (const std::exception &ex) {
        std::cout << ex.what() << std::endl;
    } catch (...) {
        std::cout << "unknow exception" << std::endl;
    }
}

Glogger::Glogger(GlogConfig config) {
    google::InitGoogleLogging(config.log_name.c_str());
    google::SetLogDestination(google::GLOG_INFO, config.file_dir.c_str());
    google::InstallFailureSignalHandler();
    google::SetLogFilenameExtension("_slam");
    // google::InstallFailureWriter(&SignalHandle);
    google::SetStderrLogging(google::ERROR);
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = config.log_size;
    FLAGS_stop_logging_if_full_disk = true;

    if (access(config.file_dir.c_str(), 0)) {
        std::cout << "folder does not exist and create: " << config.file_dir << std::endl;
        mkdir(config.file_dir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    }
}

Glogger::~Glogger() {
    google::ShutdownGoogleLogging();
}

void SignalHandle(const char *data, int size) {
    std::string txt = std::string(data, size);
    LOG(ERROR) << txt;
}



} // namespace dmlog
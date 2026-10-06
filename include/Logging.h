#pragma once

#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>

#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace ORB_SLAM3
{
// Called during construction/configuration only. The second argument selects a
// synchronous, automatically flushed logger for messages immediately before exit.
using LoggerFactory = std::function<std::shared_ptr<spdlog::logger>(const std::string &, bool)>;

inline LoggerFactory MakeLoggerFactory(LoggerFactory factory)
{
    if (factory) return factory;
    // Standalone instances share a sink across their modules, without a global
    // registry, file, or asynchronous pool. Configure it before workers start.
    const auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sink->set_pattern("[%Y-%m-%dT%H:%M:%S.%e%z] [T%t] [%n] [%^%l%$] %v");
    return [sink](const std::string &module, bool synchronous) {
        auto logger = std::make_shared<spdlog::logger>(module, sink);
        logger->set_level(spdlog::level::info);
        if (synchronous) logger->flush_on(spdlog::level::trace);
        return logger;
    };
}

inline std::shared_ptr<spdlog::logger> GetModuleLogger(
    const LoggerFactory &factory, const std::string &module, bool synchronous = false)
{
    const auto logger = factory(module, synchronous);
    // A configured factory must not silently fall back to console on failure.
    if (!logger) throw std::runtime_error("Logger factory returned null for " + module);
    return logger;
}

template<typename... Args>
inline void Log(const std::shared_ptr<spdlog::logger> &logger,
                spdlog::level::level_enum level, spdlog::string_view_t format,
                Args &&...args) noexcept
{
    // Pass values rather than preformatted strings; spdlog checks the level
    // before formatting and reports write/format failures to its error handler.
    try
    {
        logger->log(level, format, std::forward<Args>(args)...);
    }
    catch (...)
    {
        // spdlog reports unknown exceptions before rethrowing. Keep those from
        // interrupting SLAM cleanup, and never recurse into a failing logger.
        std::fprintf(stderr, "ORB-SLAM3 logging failed\n");
    }
}

// Preserve existing ostream reports while giving each line a module prefix.
// Writer runs only when enabled, inside the same nonthrowing logging boundary.
template<typename Writer>
inline void LogStream(const std::shared_ptr<spdlog::logger> &logger,
                      spdlog::level::level_enum level, Writer &&writer) noexcept
{
    if (!logger->should_log(level)) return;
    try
    {
        std::ostringstream report;
        writer(report);
        std::istringstream lines(report.str());
        std::string line;
        while (std::getline(lines, line)) Log(logger, level, "{}", line);
    }
    catch (...)
    {
        Log(logger, spdlog::level::err, "Logging report formatting failed");
    }
}

// Data-object diagnostics reuse their current Map logger. Unbound objects have
// a local console fallback, without adding logger state to serialized objects.
template<typename Writer>
inline void LogMapStream(std::shared_ptr<spdlog::logger> logger,
                         spdlog::level::level_enum level, Writer &&writer) noexcept
{
    try
    {
        if (!logger) logger = GetModuleLogger(MakeLoggerFactory({}), "map");
        LogStream(logger, level, std::forward<Writer>(writer));
    }
    catch (...)
    {
        std::fprintf(stderr, "ORB-SLAM3 logging failed\n");
    }
}
} // namespace ORB_SLAM3

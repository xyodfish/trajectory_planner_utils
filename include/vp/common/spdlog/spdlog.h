#ifndef VP_TP_SPDLOG_SHIM_H_
#define VP_TP_SPDLOG_SHIM_H_

#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace spdlog {
namespace detail {

inline void appendArgs(std::ostringstream&) {}

template <typename T, typename... Rest>
inline void appendArgs(std::ostringstream& oss, T&& arg, Rest&&... rest) {
    oss << ' ' << std::forward<T>(arg);
    appendArgs(oss, std::forward<Rest>(rest)...);
}

template <typename... Args>
inline void log(const char* level, const std::string& msg, Args&&... args) {
    std::ostringstream oss;
    oss << "[" << level << "] " << msg;
    appendArgs(oss, std::forward<Args>(args)...);
    std::cerr << oss.str() << std::endl;
}

template <typename... Args>
inline void log(const char* level, const char* msg, Args&&... args) {
    log(level, std::string(msg), std::forward<Args>(args)...);
}

}  // namespace detail

template <typename... Args>
inline void info(const std::string& msg, Args&&... args) {
    detail::log("INFO", msg, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(const char* msg, Args&&... args) {
    detail::log("INFO", msg, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(const std::string& msg, Args&&... args) {
    detail::log("WARN", msg, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(const char* msg, Args&&... args) {
    detail::log("WARN", msg, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(const std::string& msg, Args&&... args) {
    detail::log("ERROR", msg, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(const char* msg, Args&&... args) {
    detail::log("ERROR", msg, std::forward<Args>(args)...);
}

}  // namespace spdlog

#endif  // VP_TP_SPDLOG_SHIM_H_

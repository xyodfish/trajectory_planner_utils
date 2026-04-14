#ifndef __COMMON_FUNCTIONS_H__
#define __COMMON_FUNCTIONS_H__

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>
#ifdef _WIN32
#include <conio.h>  // 包含 _kbhit 和 _getch 函数
#endif
#include <mutex>

#define clamp_value(x, min, max) ((x) < min ? min : ((x) > max ? max : (x)))
/**
 * @brief 系统延时
 * 
 * @param t_start 周期开始前的当前时间戳
 * @param dt_ 需要执行的延时时间（单位：s)
 */
inline void vpSysDelay(std::chrono::steady_clock::time_point t_start, double dt_) {
    auto t_end    = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double>(t_end - t_start);
    if (duration.count() < dt_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_ - duration.count()));
    }
}

/**
 * @brief 计算当前时刻距离输入的开始时间 经过的多久（单位s)
 * 
 * @param t_start 需要判断的开始时间
 * @return double 历时时间（单位s)
 */
inline double getTimeDuration(std::chrono::steady_clock::time_point t_start) {
    auto t_now = std::chrono::steady_clock::now();
    return (std::chrono::duration_cast<std::chrono::duration<double>>(t_now - t_start)).count();
}

inline std::string getParentPath(const std::string& path) {
    std::filesystem::path fsPath(path);
    if (fsPath.has_parent_path()) {
        return fsPath.parent_path().string();
    } else {
        return "";
    }
}

inline std::string getParentPath(size_t index, const std::string& path) {
    if (index < 1) {
        return {};
    }

    std::string ret = path;

    for (int i = 0; i < index; ++i) {
        ret = getParentPath(ret);
    }

    return ret;
}

inline void printKeyDirectory(const std::vector<std::pair<std::string, std::string>>& kd) {
    std::cout << "--------------------key dirctionary  --------------------" << std::endl;
    for (const auto& p : kd) {
        std::cout << p.first << ": " << p.second << "\n";
    }
    std::cout << "-------------------------------------------------------------"
                 "-------"
              << std::endl;
}

/**
 * 计算给定向量的L2范数
 *
 * @param data 输入的向量
 * @return L2范数的值
 * @throws std::invalid_argument 如果输入向量为空
 */
inline double computeEuclideanNorm(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0;
    }

    double sum = std::accumulate(data.begin(), data.end(), 0.0,
                                 [](double acc, const double& val) { return acc + std::pow(val, 2); });
    // 返回L2范数（欧几里得范数）的平方根
    return std::sqrt(sum);
}

inline double computeAverageVal(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0;
    }

    double sum =
        std::accumulate(data.begin(), data.end(), 0.0, [](double acc, const double& val) { return acc + val; });

    return sum / data.size();
}

inline double getInterpolation(const double& lower, const double& upper, const double& ratio) {
    auto t = std::max(0.0, std::min(1.0, ratio));
    return (1.0 - t) * lower + t * upper;
}

inline size_t findVectorLowerIndex(const std::vector<double>& vec, double val) {

    auto it_lower = std::lower_bound(vec.begin(), vec.end(), val);

    return std::distance(vec.begin(), it_lower);
}

inline std::pair<int, double> calculateRatio(const double& dividend, const double& divisor) {
    // 计算商
    double result = dividend / divisor;

    int interger_result;
    double fractional_part;

    if (result > 0) {
        interger_result = (int)std::floor(result);
        fractional_part = result - interger_result;
    } else {
        interger_result = (int)std::ceil(result);
        fractional_part = -result + interger_result;
    }

    return std::make_pair(interger_result, fractional_part);
}

inline int get_char() {
    static std::mutex mtx;  // 声明一个静态互斥锁，用于保护标准输入的读取

#ifdef _WIN32
    if (_kbhit()) {
        return _getch();  // 获取并返回按键字符
    }
    return (int)('\0');
#else

    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec  = 0;
    tv.tv_usec = 10;  // 设置等待超时时间

    // 检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0) {
        std::lock_guard<std::mutex> lock(mtx);  // 锁定互斥锁
        ch = getchar();
    }
    return ch;
#endif
}

#endif

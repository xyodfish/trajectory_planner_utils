#ifndef VP_TP_RUNTIME_COMMON_H_
#define VP_TP_RUNTIME_COMMON_H_

// MSVC dll export
#ifdef USE_MSVC
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

// win/linux socket
#ifdef IS_WINDOWS_OS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <thread>
#else
#include <unistd.h>
#endif

// cross-platform sleep function
inline void vpSleepMs(int milliseconds) {
    if (milliseconds <= 0)
        return;

#ifdef IS_WINDOWS_OS
    if (milliseconds >= 3)
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    else {
        std::chrono::high_resolution_clock::time_point t_start, t_now;
        t_start = std::chrono::high_resolution_clock::now();

        while (true) {
            t_now = std::chrono::high_resolution_clock::now();
            if ((t_now - t_start).count() / 1000000.0f >= milliseconds)
                break;
        }
    }

#else
    usleep(milliseconds * 1000);
#endif
}

// cross-platform sleep function
inline void vpSleepMicroSeconds(double microseconds) {
    if (microseconds <= 0)
        return;

#ifdef IS_WINDOWS_OS
    if (microseconds >= 3000)
        std::this_thread::sleep_for(std::chrono::microseconds((int)microseconds));
    else {
        std::chrono::high_resolution_clock::time_point t_start, t_now;
        t_start = std::chrono::high_resolution_clock::now();

        while (true) {
            t_now = std::chrono::high_resolution_clock::now();
            if ((t_now - t_start).count() / 1000.0f >= microseconds)
                break;
        }
    }

#else
    usleep(microseconds);
#endif
}

#endif  // VP_TP_RUNTIME_COMMON_H_

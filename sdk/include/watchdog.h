#ifndef _watchdog_h
#define _watchdog_h

#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <functional>

namespace rp {

class Watchdog {
  public:
    Watchdog();
    Watchdog(std::function<void()> callback);
    ~Watchdog();
    void start(unsigned int _interval);
    void stop();
    void refresh();

  private:
    unsigned int mInterval;
    std::atomic<bool> mIsRunning;
    std::thread mThread;
    std::function<void()> mCallback;
    std::mutex mMutex;
    std::chrono::steady_clock::time_point mLastRefreshTime;
    std::condition_variable mStopCondition;
    void loop();
};

}

#endif /* _watchdog_h */


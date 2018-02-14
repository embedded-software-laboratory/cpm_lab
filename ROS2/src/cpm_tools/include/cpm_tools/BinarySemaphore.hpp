#pragma once
#include <mutex>
#include <condition_variable>

namespace cpm_tools { 

class BinarySemaphore {
    bool unlocked = true;
    std::mutex mtx;
    std::condition_variable cv;
public:
    void wait() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this](){return unlocked;});
        unlocked = false;
    }
    void signal() {
        {
            std::unique_lock<std::mutex> lock(mtx);
            unlocked = true;
        }
        cv.notify_one();
    }
};

}// end namespace
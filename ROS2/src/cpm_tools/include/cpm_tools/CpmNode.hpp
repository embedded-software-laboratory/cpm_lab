#pragma once

#include "cpm_tools/BinarySemaphore.hpp"
#include "cpm_tools/AbsoluteTimer.hpp"
#include "cpm_tools/Subscriber.hpp"  
#include "rclcpp/rclcpp.hpp"

#define NANOSEC_PER_SEC 1000000000ull
#define NANOSEC_PER_MILLISEC 1000000ull

namespace cpm_tools { 

uint64_t clock_gettime_nanoseconds();

class CpmNode : public rclcpp::Node {
private:
    uint64_t period_nanoseconds_; 
    uint64_t offset_nanoseconds_;
    bool allow_early_execution_;
    BinarySemaphore semaphore;
    std::vector<SubscriberBase::SharedPtr> subscribers;
    AbsoluteTimer absoluteTimer_;
public:
    CpmNode(const std::string &node_name, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool allow_early_execution);

    uint64_t getPeriodNanoseconds() {return period_nanoseconds_;}
    uint64_t getOffsetNanoseconds() {return offset_nanoseconds_;}
    bool allowEarlyExecution() {return allow_early_execution_;}

    void start_loop();

    template<typename T>
    typename cpm_tools::Subscriber<T>::SharedPtr 
    subscribe(const std::string &topic_name, uint64_t expected_delay) {
        auto sub = std::make_shared<Subscriber<T>>(topic_name, expected_delay, this, [this](){ semaphore.signal(); });
        subscribers.push_back(sub);
        return sub;
    }

    virtual void update(uint64_t deadline_nanoseconds) = 0;
    virtual ~CpmNode() {}
};

    
}// end namespace
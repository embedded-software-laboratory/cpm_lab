#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include <unistd.h>
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_tools/BinarySemaphore.hpp"
#include "cpm_tools/AbsoluteTimer.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cpm_tools;

#define NANOSEC_PER_SEC 1000000000ull

uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * NANOSEC_PER_SEC + uint64_t(t.tv_nsec);
}


class CpmNode : public rclcpp::Node {
private:
    uint64_t period_nanoseconds_; 
    uint64_t offset_nanoseconds_;
    bool allow_early_execution_;
    BinarySemaphore semaphore;
    std::vector<SubscriberBase::SharedPtr> subscribers;
    AbsoluteTimer absoluteTimer_;
public:
    CpmNode(const std::string &node_name, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool allow_early_execution)
    :Node(node_name)
    ,period_nanoseconds_(period_nanoseconds)
    ,offset_nanoseconds_(offset_nanoseconds)
    ,allow_early_execution_(allow_early_execution)
    ,absoluteTimer_(
        time_t(period_nanoseconds / NANOSEC_PER_SEC),
        long(period_nanoseconds % NANOSEC_PER_SEC),
        time_t(offset_nanoseconds / NANOSEC_PER_SEC),
        long(offset_nanoseconds % NANOSEC_PER_SEC),
        [this](){ semaphore.signal(); }
    ){}

    uint64_t getPeriodNanoseconds() {return period_nanoseconds_;}
    uint64_t getOffsetNanoseconds() {return offset_nanoseconds_;}
    bool allowEarlyExecution() {return allow_early_execution_;}

    void start_loop() {
        // Set the first deadline to the next full second plus offset
        // The period must divide one second for this to work reliably across a distributed system (TODO)
        uint64_t deadline_nanoseconds = ((clock_gettime_nanoseconds()/NANOSEC_PER_SEC)+1)*NANOSEC_PER_SEC + offset_nanoseconds_;
        while(rclcpp::ok()) {
            semaphore.wait();

            bool deadline_arrived = clock_gettime_nanoseconds() >= deadline_nanoseconds;

            bool all_messages_arrived = true;
            for(auto subscriber : subscribers) {
                if(!subscriber->message_arrived(deadline_nanoseconds)) {
                    all_messages_arrived = false;
                    break;
                }
            }

            if(deadline_arrived && !all_messages_arrived) {
                RCLCPP_WARN(this->get_logger(), "New messages missing, using old messages, in node '%s'", get_name())
            }

            if(deadline_arrived || (allow_early_execution_ && all_messages_arrived)) {
                RCLCPP_INFO(this->get_logger(), "starting run() for deadline %llu at time %llu", deadline_nanoseconds, clock_gettime_nanoseconds())
                this->update(deadline_nanoseconds);
                deadline_nanoseconds += period_nanoseconds_;
                while(clock_gettime_nanoseconds() >= deadline_nanoseconds) {
                    RCLCPP_WARN(this->get_logger(), "Missed deadline %llu", deadline_nanoseconds)
                    deadline_nanoseconds += period_nanoseconds_;
                }
            }
        }
    }

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

class ListenerNode : public CpmNode {
    Subscriber<cpm_msgs::msg::VehicleSensors>::SharedPtr subscriber_vehicle_sensors;


public:
    ListenerNode()
    :CpmNode("ListenerNode", 1 * NANOSEC_PER_SEC, 0, true)
    {
        subscriber_vehicle_sensors = subscribe<cpm_msgs::msg::VehicleSensors>("topic", 1 * NANOSEC_PER_SEC);
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        bool old_message_flag = false;
        auto msg = subscriber_vehicle_sensors->get(deadline_nanoseconds, old_message_flag);

        if(old_message_flag) {
            RCLCPP_INFO(this->get_logger(), "Old message: %i at %lld", msg.odometer_count, msg.stamp_nanoseconds)
        }
        else {
            RCLCPP_INFO(this->get_logger(), "I heard: %i at %lld", msg.odometer_count, msg.stamp_nanoseconds)
        }

    }
};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ListenerNode>();
    std::thread cpm_thread([&](){node->start_loop();});
    rclcpp::spin(node);
    rclcpp::shutdown();
    cpm_thread.join();
    return 0;
    /*rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;*/
}
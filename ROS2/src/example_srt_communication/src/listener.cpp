#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include <unistd.h>
#include "cpm_msgs/msg/vehicle_sensors.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cpm_tools;

uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}


class CpmNode : public rclcpp::Node {
private:
    uint64_t period_nanoseconds_; 
    uint64_t offset_nanoseconds_;
    bool allow_early_execution_;
public:
    CpmNode(const std::string &node_name, uint64_t period_nanoseconds, uint64_t offset_nanoseconds, bool allow_early_execution)
    :Node(node_name)
    ,period_nanoseconds_(period_nanoseconds)
    ,offset_nanoseconds_(offset_nanoseconds)
    ,allow_early_execution_(allow_early_execution){}

    uint64_t getPeriodNanoseconds() {return period_nanoseconds_;}
    uint64_t getOffsetNanoseconds() {return offset_nanoseconds_;}
    bool allowEarlyExecution() {return allow_early_execution_;}

    void start_loop() {
        while(rclcpp::ok()) {
            this->update(clock_gettime_nanoseconds());
            sleep(1);
        }
    }

    template<typename T>
    typename cpm_tools::Subscriber<T>::SharedPtr 
    subscribe(const std::string &topic_name) {
        return std::make_shared<Subscriber<T>>(topic_name, this, [&](){/*TODO notification to semaphore*/});
    }

    virtual void update(uint64_t deadline_nanoseconds) = 0;
    virtual ~CpmNode() {}
};

class ListenerNode : public CpmNode {
    Subscriber<cpm_msgs::msg::VehicleSensors>::SharedPtr subscriber_vehicle_sensors;


public:
    ListenerNode()
    :CpmNode("ListenerNode", 100 * 1000000, 10 * 1000000, true)
    {
        subscriber_vehicle_sensors = subscribe<cpm_msgs::msg::VehicleSensors>("topic");
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        for (size_t i = 0; i < subscriber_vehicle_sensors->size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "I heard: %i", subscriber_vehicle_sensors->at(i).odometer_count)
        }
        RCLCPP_INFO(this->get_logger(), "==================")
    }
};

/*class CpmNodeRunner : public rclcpp::Node {
public:
    CpmNodeRunner():Node("listener"){}

};*/



/*class Listener : public rclcpp::Node {
    Subscriber<std_msgs::msg::String, 5> sub_buf;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
        for (size_t i = 0; i < sub_buf.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", sub_buf.at(i).data.c_str())
        }
        RCLCPP_INFO(this->get_logger(), "==================")
    }

public:
    Listener() : Node("listener"), sub_buf("topic", this, [&](){RCLCPP_INFO(this->get_logger(), ".")})
    {
        timer_ = this->create_wall_timer(2s, std::bind(&Listener::timer_callback, this));
    }
};*/

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
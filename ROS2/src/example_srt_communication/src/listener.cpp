#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpm_tools/Subscriber.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cpm_tools;


class Listener : public rclcpp::Node {
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
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
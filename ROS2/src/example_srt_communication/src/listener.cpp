#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

template <typename MessageT, int buffer_size>
class SubscriptionBuffer {
    MessageT buffer[buffer_size];
    size_t latest_message_index = 0;
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
    std::function<void()> new_message_callback_;

    void topic_callback(const typename MessageT::SharedPtr msg) {
        // save messages in reverse chronologic order
        latest_message_index = (buffer_size + latest_message_index - 1) % buffer_size;
        buffer[latest_message_index] = *msg;
        if(new_message_callback_)new_message_callback_();
    }

public:
    SubscriptionBuffer (const std::string &topic_name, rclcpp::Node* node, std::function<void()> new_message_callback)
    : subscription_(node->create_subscription<MessageT>(
        topic_name, std::bind(&SubscriptionBuffer::topic_callback, this, _1), rmw_qos_profile_sensor_data
    )),
    new_message_callback_(new_message_callback)
    { }

    size_t size() { return buffer_size; }

    const MessageT& at(size_t i) {
        return buffer[(latest_message_index + i) % buffer_size];
    }
};


class Listener : public rclcpp::Node {
    SubscriptionBuffer<std_msgs::msg::String, 5> sub_buf;
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
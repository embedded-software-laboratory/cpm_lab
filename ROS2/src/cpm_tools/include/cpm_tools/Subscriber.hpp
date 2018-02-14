#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;


namespace cpm_tools { 

template <typename MessageT, int buffer_size>
class Subscriber {
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
    Subscriber (
        const std::string &topic_name, 
        rclcpp::Node* node, 
        std::function<void()> new_message_callback
    )
    : subscription_(
        node->create_subscription<MessageT>(
            topic_name, 
            std::bind(&Subscriber::topic_callback, this, _1), 
            rmw_qos_profile_sensor_data
        )
    )
    ,new_message_callback_(new_message_callback)
    { }

    size_t size() { return buffer_size; }

    const MessageT& at(size_t i) {
        // shift index so that i = 0 is the latest message
        return buffer[(latest_message_index + i) % buffer_size];
    }
};

}// end namespace
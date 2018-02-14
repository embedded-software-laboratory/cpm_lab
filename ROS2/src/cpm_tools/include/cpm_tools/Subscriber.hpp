#pragma once
#include "rclcpp/rclcpp.hpp"
#include <mutex>

namespace cpm_tools { 


class SubscriberBase {
public:
    virtual bool message_arrived(uint64_t deadline_nanoseconds) = 0;
    using SharedPtr = std::shared_ptr<SubscriberBase>;
};

template <typename MessageT, int buffer_size = 10>
class Subscriber : public SubscriberBase {
    MessageT buffer[buffer_size];
    size_t latest_message_index = 0;
    typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
    std::function<void()> new_message_callback_;
    std::mutex m_mutex;
    std::string topic_name_;
    uint64_t expected_delay_;

    void topic_callback(const typename MessageT::SharedPtr msg) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            // save messages in reverse chronologic order
            latest_message_index = (buffer_size + latest_message_index - 1) % buffer_size;
            buffer[latest_message_index] = *msg;
        }
        if(new_message_callback_)new_message_callback_();
    }

public:
    Subscriber (
        const std::string &topic_name, 
        uint64_t expected_delay,
        rclcpp::Node* node, 
        std::function<void()> new_message_callback
    )
    : subscription_(
        node->create_subscription<MessageT>(
            topic_name, 
            [this](const typename MessageT::SharedPtr msg) {topic_callback(msg);}, 
            rmw_qos_profile_sensor_data
        )
    )
    ,new_message_callback_(new_message_callback)
    ,topic_name_(topic_name)
    ,expected_delay_(expected_delay)
    { }

    size_t size() { return buffer_size; }

    MessageT at(size_t i) {
        std::unique_lock<std::mutex> lock(m_mutex);
        
        // shift index so that i = 0 is the latest message
        return buffer[(latest_message_index + i) % buffer_size];
    }

    MessageT get(uint64_t deadline_nanoseconds, bool &old_message_flag) {
        uint64_t expected_stamp_nanoseconds = deadline_nanoseconds - expected_delay_;
        std::unique_lock<std::mutex> lock(m_mutex);
        for (int i = 0; i < buffer_size; ++i)
        {
            MessageT& element = buffer[(latest_message_index + i) % buffer_size];
            if(element.stamp_nanoseconds <= expected_stamp_nanoseconds) {
                if(element.stamp_nanoseconds < expected_stamp_nanoseconds) {
                    old_message_flag = true;
                }
                return element;
            }
        }
        RCLCPP_ERROR(rclcpp::get_logger(""), "All timestamps in the topic '%s' buffer are in the future. This should never happen. Expected stamp %llu", topic_name_.c_str(), expected_stamp_nanoseconds)
        return buffer[latest_message_index];
    }

    bool message_arrived(uint64_t deadline_nanoseconds) override {
        uint64_t expected_stamp_nanoseconds = deadline_nanoseconds - expected_delay_;
        std::unique_lock<std::mutex> lock(m_mutex);
        for (int i = 0; i < buffer_size; ++i)
        {
            MessageT& element = buffer[(latest_message_index + i) % buffer_size];
            if(element.stamp_nanoseconds == expected_stamp_nanoseconds) {
                return true;
            } 
            else if (element.stamp_nanoseconds < expected_stamp_nanoseconds) {
                return false;
            }
        }
        return false;
    }

    using SharedPtr = std::shared_ptr<Subscriber>;
};

}// end namespace
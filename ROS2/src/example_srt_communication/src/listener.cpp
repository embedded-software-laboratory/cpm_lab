#include "rclcpp/rclcpp.hpp"
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include "cpm_tools/CpmNode.hpp"

using namespace std::chrono_literals;
using namespace cpm_tools;


class ListenerNode : public CpmNode {
    Subscriber<cpm_msgs::msg::VehicleSensors>::SharedPtr subscriber_vehicle_sensors;


public:
    ListenerNode()
    :CpmNode("ListenerNode", 1 * NANOSEC_PER_SEC, 0, true)
    {
        subscriber_vehicle_sensors = subscribe<cpm_msgs::msg::VehicleSensors>("topic", 300 * NANOSEC_PER_MILLISEC);
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        bool old_message_flag = false;
        auto msg = subscriber_vehicle_sensors->get(deadline_nanoseconds, old_message_flag);

        if(old_message_flag) {
            RCLCPP_INFO(this->get_logger(), "Old message: %i at %.6f", msg.odometer_count, msg.stamp_nanoseconds/double(NANOSEC_PER_SEC))
        }
        else {
            RCLCPP_INFO(this->get_logger(), "I heard: %i at %.6f", msg.odometer_count, msg.stamp_nanoseconds/double(NANOSEC_PER_SEC))
        }

    }
};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ListenerNode>();
    std::thread node_thread([&](){node->start_loop();});
    rclcpp::spin(node);
    rclcpp::shutdown();
    node_thread.join();
    return 0;
}
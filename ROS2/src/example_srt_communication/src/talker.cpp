#include "rclcpp/rclcpp.hpp"
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include "cpm_tools/CpmNode.hpp"
#include <unistd.h>

using namespace cpm_tools;

class TalkerNode : public CpmNode {    
    rclcpp::Publisher<cpm_msgs::msg::VehicleSensors>::SharedPtr publisher_;
    int count_=0;
public:
    TalkerNode()
    :CpmNode("TalkerNode", 1 * NANOSEC_PER_SEC, 700 * NANOSEC_PER_MILLISEC, false)
    {
        publisher_ = this->create_publisher<cpm_msgs::msg::VehicleSensors>("topic", rmw_qos_profile_sensor_data);
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        usleep(1000*200);
        auto message = cpm_msgs::msg::VehicleSensors();
        message.stamp_nanoseconds = deadline_nanoseconds;
        message.odometer_count = count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: %u for deadline %.6f at time %.6f", 
            message.odometer_count, 
            deadline_nanoseconds/double(NANOSEC_PER_SEC), 
            clock_gettime_nanoseconds()/double(NANOSEC_PER_SEC))
        publisher_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TalkerNode>();
    std::thread node_thread([&](){node->start_loop();});
    rclcpp::spin(node);
    rclcpp::shutdown();
    node_thread.join();
    return 0;
}
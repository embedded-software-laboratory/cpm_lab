#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <cpm_tools/AbsoluteTimer.hpp>
#include <memory>
#include "cpm_msgs/msg/vehicle_sensors.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
    Talker() : Node("talker"), count_(0) {
        publisher_ = this->create_publisher<cpm_msgs::msg::VehicleSensors>("topic", rmw_qos_profile_sensor_data);
        absoluteTimer_ = std::make_shared<AbsoluteTimer>(1,0,0,0, std::bind(&Talker::timer_callback, this));
    }

private:
    void timer_callback() {


        struct timespec t;
        clock_gettime(CLOCK_REALTIME, &t);

        auto message = cpm_msgs::msg::VehicleSensors();
        message.stamp.sec = t.tv_sec;
        message.stamp.nanosec = t.tv_nsec;
        message.odometer_count = count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: %u", message.odometer_count)
        publisher_->publish(message);
    }
    rclcpp::Publisher<cpm_msgs::msg::VehicleSensors>::SharedPtr publisher_;
    size_t count_;
    std::shared_ptr<AbsoluteTimer> absoluteTimer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
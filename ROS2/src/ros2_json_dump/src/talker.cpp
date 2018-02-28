#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpm_msgs/msg/complex_test_msg.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<cpm_msgs::msg::ComplexTestMsg>("my_test_topic");
        auto timer_callback =
            [this]() -> void {
                auto message = cpm_msgs::msg::ComplexTestMsg();

                message.my_bool = false;
                message.my_byte = 1;
                message.my_char = 'a';
                message.my_float32 = 3.14159265358979323846264338;
                message.my_float64 = -2.7182818284590452353602874713;
                message.my_int8 = -4;
                message.my_uint8 = 5;
                message.my_int16 = -6;
                message.my_uint16 = 7;
                message.my_int32 = -8;
                message.my_uint32 = 9;
                message.my_int64 = -10;
                message.my_uint64 = 11;
                message.my_string = "Hello world";


                RCLCPP_INFO(this->get_logger(), "Publishing ComplexTestMsg");
                this->publisher_->publish(message);
            };
        timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<cpm_msgs::msg::ComplexTestMsg>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
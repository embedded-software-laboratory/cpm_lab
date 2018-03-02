#include "rclcpp/rclcpp.hpp"
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_msgs/msg/vehicle_observation.hpp"
#include "cpm_msgs/msg/vehicle_command.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include "cpm_tools/CpmNode.hpp"
#include <unistd.h>

using namespace cpm_tools;

const int period_millisec = 10;

struct VehicleState { double x=0, y=0, yaw=0, speed=0, steering_angle=0; };

class SimpleSimulationNode : public CpmNode {    
    vector<string> vehicle_ids;
    map<string, VehicleState> vehicle_states;
    map<string, rclcpp::Publisher<cpm_msgs::msg::VehicleObservation>::SharedPtr> vehicle_observation_publishers;
    map<string, cpm_tools::Subscriber<cpm_msgs::msg::VehicleCommand>::SharedPtr> vehicle_command_subscribers;

public:
    SimpleSimulationNode()
    :CpmNode("SimpleSimulationNode", period_millisec * NANOSEC_PER_MILLISEC, 0, false)
    {
        // Create for each vehicle: state, observation publisher, command subscriber
        vehicle_ids = {"01", "04", "42"};
        double y = 0;
        for(auto id:vehicle_ids) {
            VehicleState s;
            s.y = y;
            vehicle_states[id] = s;

            vehicle_observation_publishers[id] = 
                this->create_publisher<cpm_msgs::msg::VehicleObservation>(
                    "vehicle" + id + "/observation", rmw_qos_profile_sensor_data
                );

            vehicle_command_subscribers[id] = subscribe<cpm_msgs::msg::VehicleCommand>(
                "vehicle" + id + "/observation", 10 * NANOSEC_PER_MILLISEC);


            y += 0.3;
        }

    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        const double dt = period_millisec / 1000.0;

        // Read inputs
        bool old_message_flag = false;
        map<string, cpm_msgs::msg::VehicleCommand> vehicle_commands;
        for(auto id:vehicle_ids) {
            vehicle_commands[id] = vehicle_command_subscribers[id]->get(deadline_nanoseconds, old_message_flag);
        }

        // Simulate controller and physics
        for(auto id:vehicle_ids) {
            auto state = vehicle_states[id];
            auto command = vehicle_commands[id];
            const double wheelbase = 0.15;
            const double T_actor_delay = 0.5;
            const double steering_angle_ref = atan(command.curvature * wheelbase);

            state.x              += dt * (state.speed * cos(state.yaw));
            state.y              += dt * (state.speed * sin(state.yaw));
            state.yaw            += dt * (state.speed * tan(state.steering_angle) / wheelbase);
            state.speed          += dt * ( (command.speed - state.speed) / T_actor_delay );
            state.steering_angle += dt * ( (steering_angle_ref - state.steering_angle) / T_actor_delay );

            vehicle_states[id] = state;
        }

        // Publish vehicle observation
        for(auto id:vehicle_ids) {
            auto state = vehicle_states[id];
            auto message = cpm_msgs::msg::VehicleObservation();
            message.pose.position.x = state.x;
            message.pose.position.y = state.y;

            message.pose.orientation.x = 0;
            message.pose.orientation.y = 0;
            message.pose.orientation.z = sin(state.yaw/2.0);
            message.pose.orientation.w = cos(state.yaw/2.0);

            message.stamp_nanoseconds = deadline_nanoseconds;

            vehicle_observation_publishers[id]->publish(message);
        }


    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSimulationNode>();
    std::thread node_thread([&](){node->start_loop();});
    rclcpp::spin(node);
    rclcpp::shutdown();
    node_thread.join();
    return 0;
}
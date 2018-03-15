#include <regex>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_msgs/msg/vehicle_observation.hpp"
#include "cpm_msgs/msg/vehicle_command.hpp"
#include "cpm_tools/Subscriber.hpp"       
#include "cpm_tools/CpmNode.hpp"

using namespace cpm_tools;

struct VehcileContext {
    int active_waypoint_index = 0;
    rclcpp::Publisher<cpm_msgs::msg::VehicleCommand>::SharedPtr command_publisher;
    cpm_tools::Subscriber<cpm_msgs::msg::VehicleObservation>::SharedPtr observation_subscriber;
};

#include "path.h"

inline double distance(double dx, double dy) { return sqrt(dx*dx + dy*dy); }

class SimpleSimulationNode : public CpmNode {
    map<string, VehcileContext> vehicles;
    int update_counter = 0;

public:
    SimpleSimulationNode()
    :CpmNode("SimpleSimulationNode", 20 * NANOSEC_PER_MILLISEC, 10 * NANOSEC_PER_MILLISEC, false)
    { }

    map<string, vector<string>> find_topics(string topic_name_regex_pattern, string required_type_name)
    {
        auto topics = get_topic_names_and_types();
        map<string, vector<string>> topic_matches;
        for(auto e:topics) {
            auto topic_name = e.first;
            for(auto type_name:e.second) {
                if(type_name == required_type_name) {
                    std::smatch m;
                    if(std::regex_match(topic_name, m, std::regex(topic_name_regex_pattern))) {
                        vector<string> groups;
                        for(auto g:m) groups.push_back(g);
                        topic_matches[topic_name] = groups;
                        break;
                    }
                }
            }
        }
        return topic_matches;
    }

    vector<string> find_vehicles() 
    {
        auto topics = find_topics("/vehicle(..)/observation", "cpm_msgs/VehicleObservation");
        vector<string> vehicle_ids;
        for(auto e:topics) {
            auto topic_name = e.first;
            if(e.second.size() == 2) {
                vehicle_ids.push_back(e.second[1]);
            }
        }
        return vehicle_ids;
    }

    void update_vehicle_connections() {
        auto vehicle_ids = find_vehicles();

        // Add new vehicles
        for(auto id:vehicle_ids) {
            if(vehicles.count(id) == 0) {
                VehcileContext ctx;

                ctx.command_publisher = create_publisher<cpm_msgs::msg::VehicleCommand>(
                    "vehicle" + id + "/command", rmw_qos_profile_sensor_data);

                ctx.observation_subscriber = subscribe<cpm_msgs::msg::VehicleObservation>(
                    "vehicle" + id + "/observation", 50 * NANOSEC_PER_MILLISEC);

                vehicles[id] = ctx;
            }
        }
    }

    void update(uint64_t deadline_nanoseconds) override 
    {
        if(update_counter % 100 == 0) update_vehicle_connections();

        for(auto& e:vehicles) {
            auto& vehicle = e.second;

            // Read input
            bool old_message_flag = false;
            auto observation = vehicle.observation_subscriber->get(deadline_nanoseconds, old_message_flag);

            const double x = observation.pose.position.x;
            const double y = observation.pose.position.y;
            const double yaw = atan2(observation.pose.orientation.z, observation.pose.orientation.w) * 2;

            // Update active path waypoint
            while(true) {
                int next_waypoint_index = (vehicle.active_waypoint_index + 1) % path_x.size();

                double waypoint_distance = distance(
                    path_x[vehicle.active_waypoint_index] - x,
                    path_y[vehicle.active_waypoint_index] - y);

                double next_waypoint_distance = distance(
                    path_x[next_waypoint_index] - x,
                    path_y[next_waypoint_index] - y);

                if(next_waypoint_distance < waypoint_distance + 1e-8) {
                    vehicle.active_waypoint_index = next_waypoint_index;
                } else {
                    break;
                }
            }

            // Controller reference values
            const double x_ref = path_x[vehicle.active_waypoint_index];
            const double y_ref = path_y[vehicle.active_waypoint_index];
            const double yaw_ref = path_yaw[vehicle.active_waypoint_index];
            const double curvature_ref = path_curvature[vehicle.active_waypoint_index];
            double lateral_error = -sin(yaw_ref) * (x-x_ref)  +cos(yaw_ref) * (y-y_ref);
            double yaw_error = sin(yaw-yaw_ref);


            lateral_error = fmin(0.9,fmax(-0.9, lateral_error));
            yaw_error = fmin(0.9,fmax(-0.9, yaw_error));

            /*
            cout 
            << "[" << "id" << ": " <<  e.first << "]"
            << "[" << "active_waypoint" << ": " <<  vehicle.active_waypoint_index << "]"
            << "[" << "lateral_error" << ": " <<  lateral_error << "]"
            << "[" << "yaw_error" << ": " <<  yaw_error << "]"
            << endl;*/

            // Linear control law
            const double curvature = curvature_ref -8.0 * lateral_error -10.0 * yaw_error;

            // Send command
            auto command = cpm_msgs::msg::VehicleCommand();
            command.stamp_nanoseconds = deadline_nanoseconds;
            command.speed = old_message_flag ? 0 : 1.0;
            command.curvature = curvature;
            vehicle.command_publisher->publish(command);
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
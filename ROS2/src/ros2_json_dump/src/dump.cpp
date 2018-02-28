#include <chrono>
#include <sstream>
#include "cpm_tools/default.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
using namespace std::chrono_literals;
using std::ostringstream;

class Dump : public rclcpp::Node
{
public:
    Dump() : Node("dump")
    {
        // Subscribe to all topics
        timer_ = this->create_wall_timer(1000ms,  [this]() -> void {
            auto topics = get_topic_names_and_types();
            for(auto e:topics) {
                auto topic_name = e.first;
                auto type_names = e.second;
                for(auto type_name:type_names) {
                    if(!(subscriptions.count(topic_name) && subscriptions[topic_name].count(type_name)))
                    {
                        subscribe(topic_name, type_name);
                    }
                }
            }
        });
    }


    void print_message(const rosidl_message_type_support_t* h, void* msg, ostringstream& out) {
        if(h && h->data) {
            auto messageMembers = (::rosidl_typesupport_introspection_cpp::MessageMembers*)(h->data);
            out << "{";
            for (size_t i = 0; i < messageMembers->member_count_; ++i) {
                if(i) out << ", ";
                auto& member = messageMembers->members_[i];
                out << "\"" << member.name_ << "\": ";
                if(!member.is_array_) {
                    void* data = msg + member.offset_;
                    if(member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64) {
                        out << *((uint64_t*)(data));
                    }
                    else if(member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32) {
                        out << *((uint32_t*)(data));
                    }
                    else if(member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32) {
                        out << *((float*)(data));
                    }
                    else if(member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64) {
                        out << *((double*)(data));
                    }
                    else if(member.type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                        print_message(member.members_, data, out);
                    }
                }
            }
            out << "}";
        }
    }

    void print_message_metainfo(const rosidl_message_type_support_t* h, void* msg, ostringstream& out, string topic_name, uint64_t receive_stamp) {
        if(h && h->data) {
            auto messageMembers = (::rosidl_typesupport_introspection_cpp::MessageMembers*)(h->data);
            out << "{";
            out << "\"topic\": \"" << topic_name << "\"";
            out << ",";
            out << "\"type\": \"" << messageMembers->package_name_ << "/" << messageMembers->message_name_ << "\"";
            out << ",";
            out << "\"receive_stamp\": " << receive_stamp;
            out << ",";
            out << "\"message\": ";
            print_message(h, msg, out);
            out << "}";
        }
    }

    template<typename T>
    void subscribe_with_type(string topic_name, string type_name) {
        subscriptions[topic_name][type_name] = this->create_subscription<T>(
            topic_name,
            [this, topic_name](typename T::UniquePtr msg) {

                const rosidl_message_type_support_t* h 
                    = rosidl_typesupport_introspection_cpp::get_message_type_support_handle<T>();

                T msg_copy = *msg;

                ostringstream out;
                print_message_metainfo(h, &msg_copy, out, topic_name, clock_gettime_nanoseconds());
                cout << out.str() << endl;
            }, rmw_qos_profile_sensor_data
        );
    }

    void subscribe(string topic_name, string type_name) {

        if(type_name == "cpm_msgs/VehicleSensors") {
            subscribe_with_type<cpm_msgs::msg::VehicleSensors>(topic_name, type_name);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    map<string, map<string, rclcpp::SubscriptionBase::SharedPtr > > subscriptions;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dump>());
    rclcpp::shutdown();
    return 0;
}
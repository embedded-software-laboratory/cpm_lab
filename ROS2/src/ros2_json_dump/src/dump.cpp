#include <chrono>
#include <sstream>
#include <limits>
#include <iomanip>
#include "cpm_tools/default.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
using namespace std::chrono_literals;
using std::ostringstream;


#include "cpm_msgs/msg/vehicle_sensors.hpp"
#include "cpm_msgs/msg/vehicle_state.hpp"
#include "cpm_msgs/msg/complex_test_msg.hpp"


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

    template<typename FloatingPointType>
    std::string convert_float(FloatingPointType value)
    {
        std::stringstream ss;
        ss << std::scientific << std::setprecision(std::numeric_limits<FloatingPointType>::digits10+1);
        ss << value;
        return ss.str();
    }

    void print_message(const rosidl_message_type_support_t* h, void* msg, ostringstream& out) {
        using namespace rosidl_typesupport_introspection_cpp;
        if(h && h->data) {
            auto messageMembers = (MessageMembers*)(h->data);
            out << "{";
            for (size_t i = 0; i < messageMembers->member_count_; ++i) {
                if(i) out << ", ";
                auto& member = messageMembers->members_[i];
                out << "\"" << member.name_ << "\": ";
                if(!member.is_array_) {
                    void* data = msg + member.offset_;
                         if(member.type_id_ == ROS_TYPE_BOOL) {out << ((*((bool*)(data)))?("true"):("false"));}
                    else if(member.type_id_ == ROS_TYPE_BYTE) {out << uint32_t(*((uint8_t*)(data)));}
                    else if(member.type_id_ == ROS_TYPE_CHAR) {out << uint32_t(*((char*)(data)));}
                    else if(member.type_id_ == ROS_TYPE_FLOAT32) {out << convert_float<float>(*((float*)(data)));}
                    else if(member.type_id_ == ROS_TYPE_FLOAT64) {out << convert_float<double>(*((double*)(data)));}
                    else if(member.type_id_ == ROS_TYPE_INT8) {out << int32_t(*((int8_t*)(data)));}
                    else if(member.type_id_ == ROS_TYPE_UINT8) {out << uint32_t(*((uint8_t*)(data)));}
                    else if(member.type_id_ == ROS_TYPE_INT16) {out << *((int16_t*)(data));}
                    else if(member.type_id_ == ROS_TYPE_UINT16) {out << *((uint16_t*)(data));}
                    else if(member.type_id_ == ROS_TYPE_INT32) {out << *((int32_t*)(data));}
                    else if(member.type_id_ == ROS_TYPE_UINT32) {out << *((uint32_t*)(data));}
                    else if(member.type_id_ == ROS_TYPE_INT64) {out << *((int64_t*)(data));}
                    else if(member.type_id_ == ROS_TYPE_UINT64) {out << *((uint64_t*)(data));}
                    else if(member.type_id_ == ROS_TYPE_STRING) {out << "\"" <<((std::string*)(data))->c_str() << "\"";}
                    else if(member.type_id_ == ROS_TYPE_MESSAGE) {print_message(member.members_, data, out);}
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

        if(type_name == "cpm_msgs/VehicleSensors") { subscribe_with_type<cpm_msgs::msg::VehicleSensors>(topic_name, type_name); }
        else if(type_name == "cpm_msgs/ComplexTestMsg") { subscribe_with_type<cpm_msgs::msg::ComplexTestMsg>(topic_name, type_name); }
        else if(type_name == "cpm_msgs/VehicleState") { subscribe_with_type<cpm_msgs::msg::VehicleState>(topic_name, type_name); }
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
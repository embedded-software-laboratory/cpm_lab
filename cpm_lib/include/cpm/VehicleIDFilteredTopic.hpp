
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

namespace cpm
{
    template<typename T>
    struct VehicleIDFilteredTopic : public dds::topic::ContentFilteredTopic<T>
    {
        VehicleIDFilteredTopic(const dds::topic::Topic<T> &topic, const uint8_t &vehicle_id)
        :dds::topic::ContentFilteredTopic<T>(
            topic,
            topic.name() + "_vehicle_id_filtered_" + std::to_string(vehicle_id), 
            dds::topic::Filter("vehicle_id = " + std::to_string(vehicle_id))
        )
        {
            static_assert(std::is_same<decltype(std::declval<T>().vehicle_id()), uint8_t>::value, "IDL type must have a vehicle_id.");
        }
    };
}
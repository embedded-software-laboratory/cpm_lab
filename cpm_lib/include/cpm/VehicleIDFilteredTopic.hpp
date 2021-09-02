#pragma once

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

namespace cpm
{
    /**
     * \class VehicleIDFilteredTopic
     * \brief Creates a DDS Topic that filters all incoming 
     * messages so that only samples with the given 
     * vehicle_id are processed further.
     * \ingroup cpmlib
     */
    template<typename T>
    struct VehicleIDFilteredTopic : public dds::topic::ContentFilteredTopic<T>
    {
        /**
         * Takes the topic which needs to be filtered. Only 
         * samples with the id vehicle_id will be considered if the 
         * filter is applied to the topic and used by e.g. a DDS 
         * Reader.
         * \param topic reference to the topic which is supposed to be used
         * \param vehicle_id reference to the vehicle id
         * \return a ContentFilteredTopic which filters the vehicle id
         */
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
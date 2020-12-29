// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

namespace cpm
{
    /**
     * \class VehicleIDFilteredTopic
     * Creates a DDS Topic that filters all incoming 
     * messages so that only samples with the given 
     * vehicle_id are processed further.
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
#pragma once

#include <dds/sub/ddssub.hpp>
#include <mutex>
#include <array>
#include <vector>
#include <map>
#include <algorithm>
#include "cpm/ParticipantSingleton.hpp"

#define CPM_READER_RING_BUFFER_SIZE (64)

namespace cpm
{
    template<typename T>
    class MultiVehicleReader
    {
    private:
        dds::sub::DataReader<T> dds_reader;
        std::mutex m_mutex;

        std::vector<size_t> buffer_indices{};
        std::vector<std::vector<T>> ring_buffers;

        std::vector<uint8_t> vehicle_ids;


        void flush_dds_reader()
        {
            auto samples = dds_reader.take();
            for(auto sample: samples)
            {
                if(sample.info().valid()) 
                {
                    uint8_t vehicle = sample.data().vehicle_id();
                    auto pos = std::distance(vehicle_ids.begin(), std::find(vehicle_ids.begin(), vehicle_ids.end(), vehicle));

                    if (pos < vehicle_ids.size() && pos >= 0) {
                        ring_buffers.at(pos).at(buffer_indices.at(pos)) = sample.data();
                        buffer_indices.at(pos) = (buffer_indices.at(pos) + 1) % CPM_READER_RING_BUFFER_SIZE;
                    }
                }
            }
        }

    public:
        MultiVehicleReader(dds::topic::Topic<T> topic, int num_of_vehicles) : 
            dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { 
            //Set size for buffers
            buffer_indices.resize(num_of_vehicles);
            ring_buffers.resize(num_of_vehicles);
            for (std::vector<T>& buffer : ring_buffers) {
                buffer.resize(CPM_READER_RING_BUFFER_SIZE);
            }

            //All buffer indices should be 0 when the program is started
            //Also: Create vehicle id list from 1 to num_of_vehicles
            for (size_t pos = 0; pos < num_of_vehicles; ++pos) {
                buffer_indices.at(pos) = 0;
                vehicle_ids.push_back(pos + 1);
            }
        }

        MultiVehicleReader(dds::topic::Topic<T> topic, std::vector<int> _vehicle_ids) : 
            dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        {             
            //Set size for buffers
            int num_of_vehicles = _vehicle_ids.size();
            buffer_indices.resize(num_of_vehicles);
            ring_buffers.resize(num_of_vehicles);
            for (std::vector<T>& buffer : ring_buffers) {
                buffer.resize(CPM_READER_RING_BUFFER_SIZE);
            }

            vehicle_ids = _vehicle_ids;

            //All buffer indices should be 0 when the program is started
            for (size_t pos = 0; pos < num_of_vehicles; ++pos) {
                buffer_indices.at(pos) = 0;
            }
        }

        MultiVehicleReader(const MultiVehicleReader &other) 
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            dds_reader = other.dds_reader;
            buffer_indices = other.buffer_indices;
            ring_buffers = other.ring_buffers;
            vehicle_ids = other.vehicle_ids;
        }
        

        void get_samples(
            const uint64_t t_now, 
            std::map<uint8_t, T>& sample_out, 
            std::map<uint8_t, uint64_t>& sample_age_out
        )
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            flush_dds_reader();

            sample_out.clear();
            sample_age_out.clear();

            for (size_t i = 0; i < vehicle_ids.size(); ++i) {
                T sample = T();
                sample.header().create_stamp().nanoseconds(0);
                sample_out[vehicle_ids.at(i)] = sample;
                sample_age_out[vehicle_ids.at(i)] = t_now;
            }

            // select samples
            for (size_t pos = 0; pos < vehicle_ids.size(); ++pos) {
                for (size_t i = 0; i < CPM_READER_RING_BUFFER_SIZE; ++i)
                {
                    auto& current_sample = 
                        ring_buffers
                        .at(pos)
                        .at(
                            (i + buffer_indices.at(pos)) % CPM_READER_RING_BUFFER_SIZE
                        );

                    if(current_sample.header().valid_after_stamp().nanoseconds() > t_now) 
                    {
                        // Data is "in the future", ignore for now
                        continue;
                    }

                    if(sample_out[vehicle_ids.at(pos)].header().create_stamp().nanoseconds() 
                                     <= current_sample.header().create_stamp().nanoseconds())
                    {
                        // Current sample has a higher timestamp, it is newer. Use it.
                        sample_out[vehicle_ids.at(pos)] = current_sample;
                        sample_age_out[vehicle_ids.at(pos)] = 
                            t_now - current_sample.header().valid_after_stamp().nanoseconds();
                    }
                }
            }
        }
    };

}
#pragma once

#include <dds/sub/ddssub.hpp>
#include <mutex>
#include <array>
#include <vector>
#include <algorithm>
#include "cpm/ParticipantSingleton.hpp"

#define CPM_READER_RING_BUFFER_SIZE (64)

namespace cpm
{
    template<typename T, std::size_t N>
    class Readers
    {
    private:
        dds::sub::DataReader<T> dds_reader;
        std::mutex m_mutex;

        std::array<size_t, N> buffer_indices{};
        std::array<std::array<T, N>, N> ring_buffers;

        std::vector<int> vehicle_ids;


        void flush_dds_reader()
        {
            auto samples = dds_reader.take();
            for(auto sample: samples)
            {
                if(sample.info().valid()) 
                {
                    int vehicle = sample.data().vehicle_id();
                    auto pos = std::distance(vehicle_ids.begin(), std::find(vehicle_ids.begin(), vehicle_ids.end(), vehicle));

                    if (pos < vehicle_ids.size() && pos >= 0) {
                        ring_buffers.at(pos).at(buffer_indices.at(pos)) = sample.data();
                        buffer_indices.at(pos) = (buffer_indices.at(pos) + 1) % CPM_READER_RING_BUFFER_SIZE;
                    }
                }
            }
        }

    public:
        Readers(dds::topic::Topic<T> &topic) : 
            dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { 
            //All buffer indices should be 0 when the program is started
            for (size_t pos = 0; pos < N, ++pos) {
                buffer_indices.at(pos) = 0;
                vehicle_ids.push_back(pos + 1);
            }
        }

        Readers(dds::topic::Topic<T> &topic, std::vector<int> _vehicle_ids) : 
            dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll()),
            vehicle_ids{_vehicle_ids}
        )
        { 
            if (_vehicle_ids.size() != N) {
                fprintf(stderr, "Error: Readers vehicle_ids size does not match template argument\n");
                fflush(stderr); 
                exit(EXIT_FAILURE);
            }

            //All buffer indices should be 0 when the program is started
            for (size_t pos = 0; pos < N, ++pos) {
                buffer_indices.at(pos) = 0;
            }
        }

        Readers(const Readers &other) 
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            dds_reader = other.dds_reader;
            ring_buffer_indices = other.ring_buffer_indices;
            ring_buffers = other.ring_buffers;
            vehicle_ids = other.vehicle_ids;
        }
        

        void get_samples(const uint64_t t_now, std::vector<T>& sample_out, std::vector<uint64_t>& sample_age_out)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            flush_dds_reader();

            sample_out.clear();
            sample_age_out.clear();

            for (size_t pos = 0; pos < N; ++pos) {
                T sample = T();
                sample.header().create_stamp().nanoseconds(0);
                sample_out.push_back(sample);
                sample_age_out.push_back(t_now);
            }

            // select samples
            for (size_t pos = 0; pos < N; ++pos) {
                for (int i = 0; i < CPM_READER_RING_BUFFER_SIZE; ++i)
                {
                    auto& current_sample = ring_buffers.get(pos).get((i + ring_buffer_index) % CPM_READER_RING_BUFFER_SIZE);

                    if(current_sample.header().valid_after_stamp().nanoseconds() > t_now) 
                    {
                        // Data is "in the future", ignore for now
                        continue;
                    }

                    if(sample_out.at(pos).header().create_stamp().nanoseconds() <= current_sample.header().create_stamp().nanoseconds())
                    {
                        // Current sample has a higher timestamp, it is newer. Use it.
                        sample_out.at(pos) = current_sample;
                        sample_age_out.at(pos) = t_now - sample_out.header().valid_after_stamp().nanoseconds();
                    }
                }
            }
        }
    };

}
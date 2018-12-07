#pragma once

#include <dds/sub/ddssub.hpp>

#include "cpm/ParticipantSingleton.hpp"

#define CPM_READER_RING_BUFFER_SIZE (32)

namespace cpm
{
    template<typename T>
    class Reader
    {
        dds::sub::DataReader<T> dds_reader;

        T ring_buffer[CPM_READER_RING_BUFFER_SIZE];
        size_t ring_buffer_index = 0; // index of next write == index of oldest element

        void flush_dds_reader()
        {
            auto samples = dds_reader.take();
            for(auto sample: samples)
            {
                if(sample.info().valid()) 
                {
                    ring_buffer[ring_buffer_index] = sample.data();
                    ring_buffer_index = (ring_buffer_index+1) % CPM_READER_RING_BUFFER_SIZE;
                }
            }
        }

    public:
        Reader(dds::topic::Topic<T> &topic)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic,
            (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { }
        
        Reader(dds::topic::ContentFilteredTopic<T> &topic)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic,
            (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { }
        

        void get_sample(const uint64_t t_now, T& sample_out, uint64_t& sample_age_out)
        {
            flush_dds_reader();

            sample_out = T();
            sample_out.header().create_stamp().nanoseconds(0);
            sample_age_out = t_now;

            // select sample
            for (int i = 0; i < CPM_READER_RING_BUFFER_SIZE; ++i)
            {
                auto& current_sample = ring_buffer[(i + ring_buffer_index) % CPM_READER_RING_BUFFER_SIZE];

                if(current_sample.header().valid_after_stamp().nanoseconds() > t_now) 
                {
                    // Data is "in the future", ignore for now
                    continue;
                }

                if(sample_out.header().create_stamp().nanoseconds() <= current_sample.header().create_stamp().nanoseconds())
                {
                    // Current sample has a higher timestamp, it is newer. Use it.
                    sample_out = current_sample;
                    sample_age_out = t_now - sample_out.header().valid_after_stamp().nanoseconds();
                }
            }
        }
    };

}
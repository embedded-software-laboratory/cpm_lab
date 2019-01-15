#pragma once

/**
 * \class Reader.hpp
 * \brief Creates a DDS Reader that, on request, returns the newest valid received sample according to the timestamp of the sample type T. Use this reader if a synchronous use of samples is desired.
 * This reader uses a ring buffer that can hold up to 64 samples. The latest 64 received samples are thus stored. All samples of type T are required to include two timestamps: A timestamp that specifies from when on they are valid (valid_after_stamp) and a stamp for when they were created (create_stamp). These stamps can be used to find out the newest sample (using create_stamp) from all 64 samples that is already valid (using valid_after_stamp) according to the current system time. Once the Reader is created, it can be used anytime to retrieve the newest valid sample, if one exist.
 */

#include <dds/sub/ddssub.hpp>
#include <mutex>
#include "cpm/ParticipantSingleton.hpp"

#define CPM_READER_RING_BUFFER_SIZE (64)

namespace cpm
{
    template<typename T>
    class Reader
    {
        dds::sub::DataReader<T> dds_reader;
        std::mutex m_mutex;

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
        /**
         * \brief Constructor using a topic to create a Reader
         * \param topic the topic of the communication
         * \return The DDS Reader
         */
        Reader(dds::topic::Topic<T> &topic)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic,
            (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { }
        
        /**
         * \brief Constructor using a filtered topic to create a Reader
         * \param topic the topic of the communication, filtered (e.g. by the vehicle ID)
         * \return The DDS Reader
         */
        Reader(dds::topic::ContentFilteredTopic<T> &topic)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic,
            (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { }

        /**
         * \brief Copy constructor using another reader
         * \param other the other reader
         * \return The copy of the other reader
         */
        Reader(const Reader &other) 
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            dds_reader = other.dds_reader;
            ring_buffer_index = other.ring_buffer_index;

            for (size_t i = 0; i < CPM_READER_RING_BUFFER_SIZE; ++i) {
                ring_buffer[i] = other.ring_buffer[i];
            }
        }
        
        /**
         * \brief get the newest valid sample that was received by the reader
         * \param t_now current system time / function call time
         * \param sample_out the new sample, if one exists
         * \param sample_age_out the age of the returned sample
         * \return This function does not directly return the sample, it is returned via the parameters
         * 
         */
        void get_sample(const uint64_t t_now, T& sample_out, uint64_t& sample_age_out)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
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
#pragma once

#include <dds/sub/ddssub.hpp>

#include "cpm/ParticipantSingleton.hpp"

namespace cpm
{
    template<typename T>
    class Reader
    {
        dds::sub::DataReader<T> dds_reader;

    public:
        Reader(dds::topic::Topic<T> &topic)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), topic,
            (dds::sub::qos::DataReaderQos() << dds::core::policy::History::KeepAll())
        )
        { }
        

        void get_sample(const uint64_t t_now, T& sample_out, uint64_t& sample_age_out)
        {
            sample_out = T();
            sample_out.header().create_stamp().nanoseconds(0);
            sample_age_out = t_now;


            // select sample

            // TODO would this be more efficient with a dds::sub::cond::QueryCondition?

            auto samples = dds_reader.read();
            for(auto sample: samples)
            {
                if(!sample.info().valid()) 
                {
                    continue;
                }

                auto sample_data = sample.data();

                if(sample_data.header().valid_after_stamp().nanoseconds() > t_now) 
                {
                    // Data is "in the future", ignore for now
                    continue;
                }

                if(sample_out.header().create_stamp().nanoseconds() <= sample_data.header().create_stamp().nanoseconds())
                {
                    // Current sample has a higher timestamp, it is newer. Use it.
                    sample_out = sample_data;
                    sample_age_out = t_now - sample_out.header().valid_after_stamp().nanoseconds();
                }
            }


            // discard old samples (older than 10 sec)
            if(t_now > 10000000000ull) // prevent unsigned underflow
            {
                const uint64_t t_discard = t_now - 10000000000ull;
                auto discarded_samples = dds_reader
                    .select()
                    .content(dds::sub::Query(dds_reader, "header.valid_after_stamp.nanoseconds < " 
                        + std::to_string(t_discard)))
                    .take();
            }            
        }
    };

}
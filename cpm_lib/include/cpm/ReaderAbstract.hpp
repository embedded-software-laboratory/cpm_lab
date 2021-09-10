#pragma once

#include <dds/sub/ddssub.hpp>
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

namespace cpm
{
    /**
     * \class ReaderAbstract
     * \brief Creates a DDS Reader that provides the simple take() function for getting all samples received after the last call of "take()"
     * Abstraction from different DDS Reader implementations
     * Difference to cpm::Reader: That one is supposed to give the latest sample w.r.t. timing information in the header. ReaderAbstract works more general than that.
     * \ingroup cpmlib
     */
    template<typename T>
    class ReaderAbstract
    {
    private:

        //! Internal DDS reader that is abstracted by this class
        dds::sub::DataReader<T> dds_reader;

        /**
         * \brief Returns qos for the settings s.t. the constructor becomes more readable
         * \param is_reliable Set the QoS to best effort / reliable
         * \param history_keep_all Set the QoS to keep the whole history / only the last message
         * \param is_transient_local Set the QoS to (not) be transient local
         */
        dds::sub::qos::DataReaderQos get_qos(bool is_reliable, bool history_keep_all, bool is_transient_local)
        {
            auto qos = dds::sub::qos::DataReaderQos();

            if (is_reliable)
            {
                qos << dds::core::policy::Reliability::Reliable();
            }
            else
            {
                //Already implicitly given
                qos << dds::core::policy::Reliability::BestEffort();
            }

            if (history_keep_all)
            {
                qos << dds::core::policy::History::KeepAll();
            }

            if (is_transient_local)
            {
                qos << dds::core::policy::Durability::TransientLocal();
            }

            return qos;
        }

    public:
        ReaderAbstract(const ReaderAbstract&) = delete;
        ReaderAbstract& operator=(const ReaderAbstract&) = delete;
        ReaderAbstract(const ReaderAbstract&&) = delete;
        ReaderAbstract& operator=(const ReaderAbstract&&) = delete;
        
        /**
         * \brief Constructor for a ReaderAbstract which is communicating within the ParticipantSingleton
         * Allows to set the topic name and some QoS settings
         * \param topic Name of the topic to read in
         * \param reliable Set the reader to be reliable (true) or use best effort (false, default)
         * \param history_keep_all Keep all received messages (true) or not (false, default)
         * \param transient_local Receive messages sent before joining (true) or not (false, default)
         */
        ReaderAbstract(std::string topic, bool reliable = false, bool history_keep_all = false, bool transient_local = false)
        :dds_reader(dds::sub::Subscriber(ParticipantSingleton::Instance()), cpm::get_topic<T>(topic), get_qos(reliable, history_keep_all, transient_local))
        { 
            
        }

        /**
         * \brief Constructor for a ReaderAbstract that communicates within another domain
         * Allows to set the topic name and some QoS settings
         * \param _participant The domain (participant) in which to read
         * \param topic Name of the topic to read in
         * \param reliable Set the reader to be reliable (true) or use best effort (false, default)
         * \param history_keep_all Keep all received messages (true) or not (false, default)
         * \param transient_local Receive messages sent before joining (true) or not (false, default)
         */
        ReaderAbstract(
            dds::domain::DomainParticipant & _participant, 
            std::string topic, 
            bool reliable = false, 
            bool history_keep_all = false, 
            bool transient_local = false
        )
        :dds_reader(dds::sub::Subscriber(_participant), cpm::get_topic<T>(_participant, topic), get_qos(reliable, history_keep_all, transient_local))
        { 
            
        }
        
        /**
         * \brief Get the received messages
         */
        std::vector<T> take()
        {
            //Only take() could be a cause for not being thread-safe, but the DDS APIs should be implemented thread-safe (is the case for RTI DDS)
            auto samples = dds_reader.take();
            std::vector<T> samples_vec;

            for (auto sample : samples)
            {
                if(sample.info().valid())
                {
                    samples_vec.push_back(sample.data());
                }
            }

            return samples_vec;
        }

        /**
         * \brief Returns # of matched writers, needs template parameter for topic type
         */
        size_t matched_publications_size()
        {
            auto matched_pub = dds::sub::matched_publications(dds_reader);
            return matched_pub.size();
        }
    };
}
#pragma once

#include <dds/core/QosProvider.hpp>
#include <dds/dds.hpp>
#include <dds/core/ddscore.hpp>

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"

namespace cpm
{
    /**
     * \class Participant
     * \brief Creates a DDS Participant, use this for abstraction
     * Also allows for loading .xml QoS files
     * \ingroup cpmlib
     */
    class Participant
    {
    private:
        //! Internal DDS participant that is abstracted by this class
        dds::domain::DomainParticipant dds_participant;

    public:
        Participant(const Participant&) = delete;
        Participant& operator=(const Participant&) = delete;
        Participant(const Participant&&) = delete;
        Participant& operator=(const Participant&&) = delete;
        
        /**
         * \brief Constructor for a participant 
         * \param domain_number Set the domain ID of the domain within which the communication takes place
         */
        Participant(int domain_number)
        :
            dds_participant(domain_number)
        { 
            
        }

        /**
         * \brief Constructor for a participant 
         * \param domain_number Set the domain ID of the domain within which the communication takes place
         * \param qos_file QoS settings to be imported from an .xml file
         */
        Participant(int domain_number, std::string qos_file)
        :
            dds_participant(domain_number, dds::core::QosProvider(qos_file).participant_qos())
        { 
            
        }

        /**
         * \brief Constructor for a participant 
         * \param domain_number Set the domain ID of the domain within which the communication takes place
         * \param qos_file QoS settings to be imported from an .xml file
         * \param qos_profile QoS profile from the .xml file that should be applied
         */
        Participant(int domain_number, std::string qos_file, std::string qos_profile)
        :
            dds_participant(domain_number, dds::core::QosProvider(qos_file, qos_profile).participant_qos())
        { 
            
        }

        /**
         * \brief Constructor for a participant 
         * \param participant A dds participant to be stored in this wrapper function (only for the middleware, replace after eProsima implementation)
         */
        Participant(dds::domain::DomainParticipant& participant)
        :
            dds_participant(participant)
        { 
            
        }
        
        /**
         * \brief Function to get the internally stored DDS-representation of the participant
         */
        dds::domain::DomainParticipant& get_participant()
        {
            return dds_participant;
        }
    };
}

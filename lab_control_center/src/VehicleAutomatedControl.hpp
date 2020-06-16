#pragma once

#include "defaults.hpp"
#include <dds/pub/ddspub.hpp>
#include "VehicleCommandSpeedCurvature.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/stamp_message.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/TimerFD.hpp"

/**
 * \brief This class is used to send automated control structures to the vehicles. A prominent example would be a stop signal that is sent to 
 * all vehicles after a simulation was stopped, so that they try to freeze at their current position and do not 'drive on while slowing down'
 * 
 */
class VehicleAutomatedControl
{
private:
    //DDS data structures to send automated commands to the vehicles
    dds::domain::DomainParticipant& participant;
    dds::topic::Topic<VehicleCommandSpeedCurvature> topic_vehicleCommandSpeedCurvature;
    shared_ptr<dds::pub::DataWriter<VehicleCommandSpeedCurvature>> writer_vehicleCommandSpeedCurvature = nullptr;

    //Vehicle commands need to be sent regularly to be interpreted correctly, so e.g. a stop signal should not be sent only once (TODO: Check that)
    std::shared_ptr<cpm::TimerFD> task_loop = nullptr;
    std::mutex stop_list_mutex;
    std::map<uint32_t, uint32_t> vehicle_stop_list; //TODO: Unordered set / different approach

public:
    VehicleAutomatedControl();

    /**
     * \brief This function is used to send an immediate stop signal to all vehicles
     * \param id_list List of vehicle ids of the vehicles that should be stopped
     */
    void stop_vehicles(std::vector<uint8_t> id_list);

    /**
     * \brief This function is used to send an immediate stop signal to a single vehicle
     * \param id Vehicle id
     */
    void stop_vehicle(uint8_t id);
};
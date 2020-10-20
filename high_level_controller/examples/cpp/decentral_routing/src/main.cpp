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


#include "lane_graph.hpp"                       //sw-folder central routing->include
#include "lane_graph_tools.hpp"                 //sw-folder central routing

// CPM Wrappers and functions
#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/MultiVehicleReader.hpp"           //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "cpm/get_topic.hpp"

// IDL files
#include "VehicleObservation.hpp" 
#include "VehicleCommandTrajectory.hpp"
#include "TimeStamp.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"
#include "VehicleStateList.hpp"

// DDS files
#include <dds/pub/ddspub.hpp>                   //rti folder
#include <dds/sub/ddssub.hpp>                   //rti folder
#include <dds/domain/DomainParticipant.hpp>         // Required for communication with middleware
#include <dds/core/QosProvider.hpp>             // Required for communication with middleware

// General C++ libs
#include <chrono>
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>

// Planner
#include "VehicleTrajectoryPlanner.hpp"    //sw-folder central routing

using std::vector;

int main(int argc, char *argv[]) {   

    // Read command line arguments
    const bool middleware_enabled = cpm::cmd_parameter_bool("middleware", false, argc, argv); //variable is set to false 
    const bool simulated_time_enabled = cpm::cmd_parameter_bool("simulated_time", false, argc, argv); //variable is set to false 
    const std::vector<int> vehicle_ids_int = cpm::cmd_parameter_ints("vehicle_ids", {4}, argc, argv);

    cpm::init(argc, argv);

    // Validate given vehicle id(s)
    uint8_t vehicle_id;

    assert(vehicle_ids_int.size()==1);

    std::vector<uint8_t> vehicle_ids;
    for(auto i:vehicle_ids_int)
    {
        assert(i>0);
        assert(i<255);
        vehicle_id = i;
        vehicle_ids.push_back(i);
    }
    cpm::Logging::Instance().set_id("decentral_routing_"+std::to_string(vehicle_id));

    cpm::Logging::Instance().write(3,
            "decentral_routing_%d coming online",
            vehicle_id
    );


    // Outstream in shell which vehicles were selected
    std::stringstream vehicle_id_stream;
    vehicle_id_stream << "Started HLC for Vehicle ID: ";
    vehicle_id_stream << static_cast<uint32_t>(vehicle_id); //Cast s.t. uint8_t is not interpreted as a character
    std::string vehicle_id_string = vehicle_id_stream.str();

    std::cout << vehicle_id_string << std::endl;

    // Initialize Planner and constants necessary for planning
    // Definition of a timesegment in nano seconds and a trajecotry planner for more than one vehicle
    const uint64_t dt_nanos = 400000000ull;
    std::unique_ptr<VehicleTrajectoryPlanner> planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(dt_nanos));
    cpm::Logging::Instance().write(3,
            "Planner created");

    // Initialize everything needed for communication with middleware
    const int dds_domain = cpm::cmd_parameter_int("dds_domain", 1, argc, argv);
    // QoS for comms between middleware and HLC, on the same machine (local)
    dds::core::QosProvider local_comms_qos_provider("QOS_LOCAL_COMMUNICATION.xml");
    dds::domain::DomainParticipant local_comms_participant(dds_domain, local_comms_qos_provider.participant_qos());
    dds::pub::Publisher local_comms_publisher(local_comms_participant);
    dds::sub::Subscriber local_comms_subscriber(local_comms_participant);

    /* --------------------------------------------------------------------------------- 
     * Create readers and writers for communication with middleware
     * ---------------------------------------------------------------------------------
     */
    // These QoS Settings are taken from the QOS_READY_TRIGGER.xml used in matlab
    dds::pub::DataWriter<ReadyStatus> writer_readyStatus(
            local_comms_publisher,
            cpm::get_topic<ReadyStatus>(local_comms_participant, "readyStatus"),
            (dds::pub::qos::DataWriterQos()
                << dds::core::policy::Reliability::Reliable()
                << dds::core::policy::History::KeepAll()
                << dds::core::policy::Durability::TransientLocal())
    );

    // systemTrigger Reader, QoS Settings taken from QOS_READY_TRIGGER.xml
    dds::sub::DataReader<SystemTrigger> reader_systemTrigger(
            local_comms_subscriber,
            cpm::get_topic<SystemTrigger>(local_comms_participant, "systemTrigger"),
            (dds::sub::qos::DataReaderQos()
                << dds::core::policy::Reliability::Reliable()
                << dds::core::policy::History::KeepAll())
    );

    // This reader might be unnecessary; gets other vehicles' states from middleware
    dds::sub::DataReader<VehicleStateList> reader_vehicleStateList(
            local_comms_subscriber,
            cpm::get_topic<VehicleStateList>(
                local_comms_participant,
                "vehicleStateList")
    );
     
    // Writer to send trajectory to middleware
    dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
        local_comms_publisher,
        cpm::get_topic<VehicleCommandTrajectory>(
            local_comms_participant,
            "vehicleCommandTrajectory")
    );

    // Reader to get position of vehicle from lab camera; determines start position
    //dds::sub::DataReader<VehicleObservation> reader_vehicleObservation(
    //        dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()),
    //        cpm::get_topic<VehicleObservation>("vehicleObservation")
    //);
    
    cpm::MultiVehicleReader<VehicleObservation> ips_reader(
        cpm::get_topic<VehicleObservation>("vehicleObservation"),
        5
    );

    // Writer to communicate plans with other vehicles
    dds::pub::DataWriter<LaneGraphTrajectoryChanges> writer_laneGraphTrajectoryChanges(
            dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
            cpm::get_topic<LaneGraphTrajectoryChanges>("laneGraphTrajectoryChanges")
    );

    // Reader to receive planned trajectories of other vehicles
    dds::sub::DataReader<LaneGraphTrajectoryChanges> reader_laneGraphTrajectoryChanges(
            dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
            cpm::get_topic<LaneGraphTrajectoryChanges>("laneGraphTrajectoryChanges")
    );
    //std::function<void()> callback_func([planner](dds::sub::LoanedSamples<LaneGraphTrajectoryChanges> samples){
    //        planner->process_samples(samples);
    //        });
    //std::function<void(dds::sub::LoanedSamples<LaneGraphTrajectoryChanges>)> callback_func(planner->process_samples);
    //cpm::AsyncReader<LaneGraphTrajectoryChanges> reader_laneGraphTrajectoryChanges(
    //        callback_func,
    //        cpm::ParticipantSingleton::Instance(),
    //        cpm::get_topic<LaneGraphTrajectoryChanges>("laneGraphTrajectoryChanges")
    //);

    /* ---------------------------------------------------------------------------------
     * Send/receive initial LaneGraphTrajectories
     * ---------------------------------------------------------------------------------
     */
    //TODO

    /* ---------------------------------------------------------------------------------
     * Compose and send Ready message
     * ---------------------------------------------------------------------------------
     */
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // Create arbitrary timestamp as per ReadyStatus.idl
    TimeStamp timestamp(11111);
    // The middleware expects a message like "hlc_${vehicle_id}", e.g. hlc_1
    std::string hlc_identification("hlc_");
    hlc_identification.append(std::to_string(vehicle_id));
    ReadyStatus readyStatus(hlc_identification, timestamp);
    writer_readyStatus.write(readyStatus);

    /* ---------------------------------------------------------------------------------
     * Wait until we receive systemTrigger; then we start
     * ---------------------------------------------------------------------------------
     */
    bool break_while = false;
    while(!break_while) {
        dds::sub::LoanedSamples<SystemTrigger> samples = reader_systemTrigger.take();
        for(auto sample : samples) {
            if (sample.info().valid()) {
                std::cout << "Received SystemTrigger, starting" << std::endl;
                break_while = true;
                break; 
            }
        }
    }

    /* ---------------------------------------------------------------------------------
     * Start timer
     * ---------------------------------------------------------------------------------
     */
    auto timer = cpm::Timer::create("decentral_routing", dt_nanos, 0, false, true, simulated_time_enabled); 
    timer->start([&](uint64_t t_now)
    {
        dds::sub::LoanedSamples<SystemTrigger> samples = reader_systemTrigger.take();
        // Check for stop condition 
        for(auto sample : samples) {
            if (sample.info().valid()) {
                //TODO: Act, when we receive a SystemTrigger signal
                std::cout << "Unhandled SystemTrigger" << std::endl;
            }
        }
        planner->set_real_time(t_now);

        if(planner->is_started())//will be set to true after fist activation
        {
            auto computation_start_time = timer->get_time();
            //get trajectory commands from VehicleTrajectoryPlanner with new points
            auto command = planner->get_trajectory_command(t_now);
            auto computation_end_time = timer->get_time();

            cpm::Logging::Instance().write(
                3,
                "%s, Computation start time: %llu, Computation end time: %llu",
                std::to_string(vehicle_id), computation_start_time, computation_end_time
            );
            
            writer_vehicleCommandTrajectory.write(command);
        }
        else //prepare to start planner
        {
            cpm::Logging::Instance().write(3,
                    "Preparing to start planner");
            // reset planner object
            planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(dt_nanos));
            planner->set_real_time(t_now);

            /* -------------------------------------------------------------------------
             * Check for start position of our vehicle
             * -------------------------------------------------------------------------
             */
            bool matched = false;
            while(!matched) {
                //dds::sub::LoanedSamples<VehicleObservation> samples =
                //    reader_vehicleObservation.take();
                std::map<uint8_t, VehicleObservation> ips_sample;
                std::map<uint8_t, uint64_t> ips_sample_age;
                ips_reader.get_samples(t_now, ips_sample, ips_sample_age);

                for(auto sample:ips_sample)
                {
                    auto data = sample.second;
                    if( vehicle_id == data.vehicle_id()) {
                        auto pose = data.pose();
                        int out_edge_index = -1;
                        int out_edge_path_index = -1;
                        bool matched = laneGraphTools.map_match_pose(pose, out_edge_index, out_edge_path_index);
                        //if vehicle was found on map, add vehicle to MultiVehicleTrajectoryPlanner
                        if(matched)
                        {
                            planner->set_vehicle(std::make_shared<VehicleTrajectoryPlanningState>(vehicle_id, out_edge_index, out_edge_path_index));
                            cpm::Logging::Instance().write(
                                3,
                                "Vehicle %d matched.",
                                int(vehicle_id)
                            );
                        }
                        else //Errormessage, if not all vehicles could be matched to the map
                        {
                            cpm::Logging::Instance().write(
                                1,
                                "Error: Vehicle %d not matched.",
                                int(vehicle_id)
                            );
                        }
                    }
                }
            }

            planner->set_writer(
                std::make_shared<dds::pub::DataWriter<LaneGraphTrajectoryChanges>>(
                    writer_laneGraphTrajectoryChanges
                    )
                );
            planner->set_reader(
                std::make_shared<dds::sub::DataReader<LaneGraphTrajectoryChanges>>(
                    reader_laneGraphTrajectoryChanges
                    )
                );
            //planner->init_reader(cpm::ParticipantSingleton::Instance());

            //Start the Planner. That includes collision avoidance. In this case we avoid collisions by priority assignment
            //with the consequence of speed reduction for the lower prioritized vehicle (here: Priority based on descending vehicle ID of the neighbours.)
            planner->start();
        }
    });
}

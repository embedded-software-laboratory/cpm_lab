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

// Set to true to get additional information about execution time in stdout
#define TIMED true

#include "lane_graph.hpp"                       //sw-folder central routing->include
#include "lane_graph_tools.hpp"                 //sw-folder central routing

// CPM Wrappers and functions
#include "cpm/Logging.hpp"                      //->cpm_lib->include->cpm
#include "cpm/CommandLineReader.hpp"            //->cpm_lib->include->cpm
#include "cpm/init.hpp"                         //->cpm_lib->include->cpm
#include "cpm/ParticipantSingleton.hpp"         //->cpm_lib->include->cpm
#include "cpm/Timer.hpp"                        //->cpm_lib->include->cpm
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"
#include "cpm/ReaderAbstract.hpp"

// IDL files
#include "VehicleCommandTrajectory.hpp"
#include "TimeStamp.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"
#include "VehicleStateList.hpp"
#include "StopRequest.hpp"

// General C++ libs
#include <chrono>
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <limits>                           // To get maximum integer value (for stop condition)

#include "CouplingGraph.hpp"
// Planner
#include "VehicleTrajectoryPlanner.hpp"    //sw-folder central routing

using std::vector;

int main(int argc, char *argv[]) {   

    // Read command line arguments
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

    // SystemTrigger value that means "stop" (as defined in SystemTrigger.idl)
    const uint64_t trigger_stop = std::numeric_limits<uint64_t>::max();

    // FIXME: Dirty hack to get our QOS settings
    // On the NUC we only have the QOS File for the middleware
    // and RTI DDS doesn't want to load it from there, so we copy it to our working dir.
    system("cp $HOME/dev/software/middleware/build/QOS_LOCAL_COMMUNICATION.xml .");
    //TODO: QOS_READY_TRIGGER xml is currently only local, not on the NUC - right?

    // Initialize everything needed for communication with middleware
    const int middleware_domain = cpm::cmd_parameter_int("middleware_domain", 1, argc, argv);
    //dds::core::QosProvider local_comms_qos_provider("./QOS_LOCAL_COMMUNICATION.xml", "MatlabLibrary::LocalCommunicationProfile");
    //dds::domain::DomainParticipant local_comms_participant(
    //        middleware_domain,
    //        local_comms_qos_provider.participant_qos()
    //);
    cpm::Participant local_comms_participant(
            middleware_domain,
            "./QOS_LOCAL_COMMUNICATION.xml",
            "MatlabLibrary::LocalCommunicationProfile"
    );

    /* --------------------------------------------------------------------------------- 
     * Create readers and writers for communication with middleware
     * ---------------------------------------------------------------------------------
     */
    // These QoS Settings are taken from the QOS_READY_TRIGGER.xml used in matlab example
    cpm::Writer<ReadyStatus> writer_readyStatus(
            local_comms_participant.get_participant(),
            "readyStatus",
            true,
            true,
            true
    );

    //dds::pub::DataWriter<ReadyStatus> writer_readyStatus(
    //        local_comms_publisher,
    //        cpm::get_topic<ReadyStatus>(local_comms_participant, "readyStatus"),
    //        (local_comms_qos_provider.datawriter_qos()
    //            << dds::core::policy::Reliability::Reliable()
    //            << dds::core::policy::History::KeepAll()
    //            << dds::core::policy::Durability::TransientLocal())
    //);

    // systemTrigger Reader, QoS Settings taken from QOS_READY_TRIGGER.xml
    cpm::ReaderAbstract<SystemTrigger> reader_systemTrigger(
            local_comms_participant.get_participant(),
            "systemTrigger",
            true,
            true
    );

    //dds::sub::DataReader<SystemTrigger> reader_systemTrigger(
    //        local_comms_subscriber,
    //        cpm::get_topic<SystemTrigger>(local_comms_participant, "systemTrigger"),
    //        (dds::sub::qos::DataReaderQos()
    //            << dds::core::policy::Reliability::Reliable()
    //            << dds::core::policy::History::KeepAll())
    //);

    // VehicleStateList is our timing signal from the middleware
    cpm::ReaderAbstract<VehicleStateList> reader_vehicleStateList(
            local_comms_participant.get_participant(),
            "vehicleStateList"
    );

    //dds::sub::DataReader<VehicleStateList> reader_vehicleStateList(
    //        local_comms_subscriber,
    //        cpm::get_topic<VehicleStateList>(
    //            local_comms_participant,
    //            "vehicleStateList")
    //);
     
    // Writer to send trajectory to middleware
    cpm::Writer<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
            local_comms_participant.get_participant(),
            "vehicleCommandTrajectory"
    );

    //dds::pub::DataWriter<VehicleCommandTrajectory> writer_vehicleCommandTrajectory(
    //    local_comms_publisher,
    //    cpm::get_topic<VehicleCommandTrajectory>(
    //        local_comms_participant,
    //        "vehicleCommandTrajectory")
    //);

    // Writer to send a StopRequest to LCC (in case of failure)
    //dds::pub::DataWriter<StopRequest> writer_stopRequest(
    //        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
    //        cpm::get_topic<StopRequest>("stopRequest")
    //);
    cpm::Writer<StopRequest> writer_stopRequest("stopRequest");

    /* 
     * Reader/Writers for comms between vehicles directly
     */
    // Writer to communicate plans with other vehicles
    cpm::Writer<HlcCommunication> writer_HlcCommunication(
            "hlcCommunication");

    // Reader to receive planned trajectories of other vehicles
    cpm::ReaderAbstract<HlcCommunication> reader_HlcCommunication(
            "hlcCommunication");
    
    /* ---------------------------------------------------------------------------------
     * Create planner object
     * ---------------------------------------------------------------------------------
     */
    auto planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner());

    // Set reader/writers of planner so it can communicate with other planners
    planner->set_writer(
    std::unique_ptr<cpm::Writer<HlcCommunication>>(
        new cpm::Writer<HlcCommunication>("hlcCommunication")
        )
    );
    planner->set_reader(
    std::unique_ptr<cpm::ReaderAbstract<HlcCommunication>>(
        new cpm::ReaderAbstract<HlcCommunication>("hlcCommunication")
        )
    );

    /* ---------------------------------------------------------------------------------
     * Compose and send Ready message
     * ---------------------------------------------------------------------------------
     */
    // TODO: Why do we need this 5 seconds wait? We do, but why and what would be a better solution?
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // Create arbitrary timestamp as per ReadyStatus.idl
    TimeStamp timestamp(11111);
    // The middleware expects a message like "hlc_${vehicle_id}", e.g. hlc_1
    std::string hlc_identification("hlc_");
    hlc_identification.append(std::to_string(vehicle_id));
    ReadyStatus readyStatus(hlc_identification, timestamp);
    writer_readyStatus.write(readyStatus);

    /* ---------------------------------------------------------------------------------
     * main loop to regularly send commands
     * ---------------------------------------------------------------------------------
     */
    bool matched = false; // Status if we initialized vehicle on the LaneGraph
    bool planning = false; // Status if we are currently planning
    bool new_vehicleStateList = false; // If we received a new VehicleStateList
    bool stop = false; // Exit from main loop
    VehicleStateList vehicleStateList;
    std::future<std::unique_ptr<VehicleCommandTrajectory>> cmd_future;

#if TIMED
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = std::chrono::steady_clock::now();
#endif

    while( !stop ) {
        auto state_samples = reader_vehicleStateList.take();
        for(auto sample : state_samples) {
            // We received a StateList, which is our timing signal
            // to send commands to vehicle
            new_vehicleStateList = true;
            vehicleStateList = sample;
        }

        // The first time we receive a sample, we need to initialize our position on the laneGraph
        if(new_vehicleStateList && !planner->is_started()) {
            cpm::Logging::Instance().write(3,
                "Preparing to start planner");

            /* ---------------------------------------------------------------------
            * Check for start position of our vehicle
            * ----------------------------------------------------------------------
            */
            // This probably does not require a loop,
            // we just need to find the one VehicleState with our vehicle_id
            for(auto vehicle_state : vehicleStateList.state_list())
            {
                if( vehicle_id == vehicle_state.vehicle_id() ) {
                    auto pose = vehicle_state.pose();
                    int out_edge_index = -1;
                    int out_edge_path_index = -1;
                    matched = laneGraphTools.map_match_pose(pose, out_edge_index, out_edge_path_index);
                    if( !matched ) {
                        cpm::Logging::Instance().write(1,
                            "Couldn't find starting position,\
                            try moving the vehicle if this persists.");
                    } else {

                        // This graphs gives the priorities, as well as the order of planning
                        // Currently we only plan sequentially, with lower vehicle ids first
                        std::vector<int> vec(vehicleStateList.active_vehicle_ids());
                        CouplingGraph coupling_graph(vec);
                        // For testing, make all planning one iterative block
                        coupling_graph.addIterativeBlock(std::vector<int>( vec.begin(), vec.end()));

                        planner->set_coupling_graph(coupling_graph);

                        // Initialize PlanningState with starting position
                        planner->set_vehicle(
                                std::unique_ptr<VehicleTrajectoryPlanningState>(
                                    new VehicleTrajectoryPlanningState(
                                        vehicle_id,
                                        out_edge_index,
                                        out_edge_path_index
                                        )
                                )
                        );
                        cpm::Logging::Instance().write(
                                2,
                                "Vehicle %d matched.",
                                int(vehicle_id)
                        );
                    }
                }
            }

            if( !matched ) {
                // We might want to stop everything here, but it happens too
                // often for that
                cpm::Logging::Instance().write(2,
                    "Couldn't find vehicle in VehicleStateList.");
            }
        }

        if( new_vehicleStateList && matched ) {
            // We are using the stateList now, so it isn't new anymore
            new_vehicleStateList = false;

            // Stop planning of previous timestep, if necessary
            if(planning) {
#if TIMED
                end_time = std::chrono::steady_clock::now();
                auto diff = end_time - start_time;
                std::cout
                    << "TIMING: Aborting planning after "
                    << std::chrono::duration<double, std::milli>(diff).count()
                    << " ms"
                    << std::endl;
#endif
                planner->stop();
                planning = false;
            }

#if TIMED
            std::cout
                << "------------- Timestep "
                << vehicleStateList.t_now()
                << " -------------"
                << std::endl;
            start_time = std::chrono::steady_clock::now();
#endif
            // Start async job for planning
            cmd_future = std::async(std::launch::async,
                    [&]{
                        return planner->plan(vehicleStateList.t_now(), vehicleStateList.period_ms()*1e6);
                    }
                );
            planning = true;
        }

        // Check status of async planning
        if( planning && cmd_future.valid() ) {
            // Waits 1ms for result (or just get it instantly?)
            std::future_status future_status = cmd_future.wait_for(std::chrono::milliseconds(1));
            if( future_status == std::future_status::ready ) {
#if TIMED
                end_time = std::chrono::steady_clock::now();
                auto diff = end_time - start_time;
                std::cout
                    << "TIMING: Finished planning after "
                    << std::chrono::duration<double, std::milli>(diff).count()
                    << " ms"
                    << std::endl;
#endif
                // Get commands and send to vehicle
                std::unique_ptr<VehicleCommandTrajectory> cmd = cmd_future.get();
                if( cmd.get() != nullptr ) {
                    writer_vehicleCommandTrajectory.write(*cmd.get());
                } else {
                    cpm::Logging::Instance().write(2,
                            "Unexpected missing value from planner");
                }
                planning = false;
            }
        }

        // Check if we received a SystemTrigger to stop
        auto systemTrigger_samples = reader_systemTrigger.take();
        for (auto sample : systemTrigger_samples) {
            if (sample.next_start().nanoseconds() == trigger_stop) {
                cpm::Logging::Instance().write(
                    2,
                    "Received stop signal, stopping"
                    );
                    stop = true;
            }
        }

        // Check if the planner encountered a problem
        if( planner->is_crashed() ) {
            stop = true;
            StopRequest request(vehicle_id);
            writer_stopRequest.write(request);
        }

        // Rate limit main while loop to 10-times per dt_nanos
        // Arbitrary, but larger waits should decrease cpu load of this loop
        std::this_thread::sleep_for(std::chrono::nanoseconds(vehicleStateList.period_ms()/10));
    }

    return 0;
}

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
#include <limits>                           // To get maximum integer value (for stop condition)

// Planner
#include "VehicleTrajectoryPlanner.hpp"    //sw-folder central routing

using std::vector;

int main(int argc, char *argv[]) {   

    // Read command line arguments
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
    // Definition of time stamp at which to stop
    const uint64_t trigger_stop = std::numeric_limits<uint64_t>::max();

    // Create a dummy planner, so it's initialized
    auto planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(0));
    planner->set_real_time(0);

    // Initialize everything needed for communication with middleware
    const int middleware_domain = cpm::cmd_parameter_int("middleware_domain", 1, argc, argv);

    // FIXME: Dirty hack to get our QOS settings
    // Currently we only have the middleware QOS File on the NUC, so we use that
    std::cout << system("cp $HOME/dev/software/middleware/build/QOS_LOCAL_COMMUNICATION.xml .");

    // For some reason we cannot access a QOS file outside our working directory
    dds::core::QosProvider local_comms_qos_provider("./QOS_LOCAL_COMMUNICATION.xml");
    dds::domain::DomainParticipant local_comms_participant(middleware_domain, local_comms_qos_provider.participant_qos());
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

    // Writer to communicate plans with other vehicles
    dds::pub::DataWriter<LaneGraphTrajectory> writer_laneGraphTrajectory(
            dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
            cpm::get_topic<LaneGraphTrajectory>("laneGraphTrajectory")
    );

    // Reader to receive planned trajectories of other vehicles
    dds::sub::DataReader<LaneGraphTrajectory> reader_laneGraphTrajectory(
            dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), 
            cpm::get_topic<LaneGraphTrajectory>("laneGraphTrajectory")
    );

    /* ---------------------------------------------------------------------------------
     * Send/receive initial LaneGraphTrajectories
     * ---------------------------------------------------------------------------------
     */
    //TODO

    /* ---------------------------------------------------------------------------------
     * Compose and send Ready message
     * ---------------------------------------------------------------------------------
     */
    // TODO: Why do we need this 5 seconds wait?
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
    /* Soll-Verhalten:
     * - Wir starten erst, wenn eine StateList von der Middleware kommt
     *   - Wir schicken dann auch erst VehicleTrajectories an die Middleware, wenn eine
     *   weitere StateList kommt
     */
    bool got_start = false;
    while(!got_start) {
        dds::sub::LoanedSamples<SystemTrigger> samples = reader_systemTrigger.take();
        for(auto sample : samples) {
            if (sample.info().valid()) {
                uint64_t next_start = sample.data().next_start().nanoseconds();
                if ( next_start == trigger_stop ) {
                    std::cout << "Received stop signal" << std::endl;
                    cpm::Logging::Instance().write(
                        3,
                        "HLC of vehicle %s received stop signal",
                        std::to_string(vehicle_id)
                    );
                    // Abort HLC
                    return 0;
                } else if ( next_start ) {
                    std::cout << "Received start signal" << std::endl;
                    cpm::Logging::Instance().write(
                        3,
                        "HLC of vehicle %s received start signal",
                        std::to_string(vehicle_id)
                    );
                    got_start = true;
                }
            }
        }
    }

    /* ---------------------------------------------------------------------------------
     * Start timer
     * ---------------------------------------------------------------------------------
     */
    uint64_t t_now = 0;
    uint64_t dt_nanos = 0;
    bool stopped = false;
    while (!stopped) {
        // Check the systemTrigger Topic for stop signal
        // This might be unnecessary when using the middleware
        dds::sub::LoanedSamples<SystemTrigger> trigger_samples = reader_systemTrigger.take();
        for(auto sample : trigger_samples) {
            if (sample.info().valid()) {
                uint64_t next_start = sample.data().next_start().nanoseconds();
                // Stop the HLC when we receive a stop signal
                // Probably superfluous, since the timer stops by itself on
                // stop signals.
                if( next_start == trigger_stop ) {
                    std::cout << "Received stop signal" << std::endl;
                    cpm::Logging::Instance().write(
                        3,
                        "HLC of vehicle %s received stop signal",
                        std::to_string(vehicle_id)
                    );
                    stopped=true;
                }
            }
        }

    dds::sub::LoanedSamples<VehicleStateList> state_samples = reader_vehicleStateList.take();
    for(auto sample : state_samples) {
        if( sample.info().valid() ) {
            t_now = sample.data().t_now();
            dt_nanos = sample.data().period_ms()*1e6;

            if( dt_nanos != 400000000ull ) {
                cpm::Logging::Instance().write(
                        1,
                        "Please set middleware_period_ms to 400ms")
            }

            if(planner->is_started())//will be set to true after fist activation
            {
                planner->set_real_time(t_now);
                //get trajectory commands from VehicleTrajectoryPlanner with new points
                auto command = planner->get_trajectory_command(t_now);

                writer_vehicleCommandTrajectory.write(command);
            }
            else //prepare to start planner
            {
                cpm::Logging::Instance().write(3,
                    "Preparing to start planner");
                // reset planner object
                planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(dt_nanos));
                planner->set_real_time(t_now);

                /* ---------------------------------------------------------------------
                * Check for start position of our vehicle
                * ----------------------------------------------------------------------
                */
                bool matched = false;
                //FIXME: This probably does not require a loop
                for(auto vehicle_state : sample.data().state_list())
                {
                    if( vehicle_id == vehicle_state.vehicle_id() ) {
                    auto pose = vehicle_state.pose();
                    int out_edge_index = -1;
                    int out_edge_path_index = -1;
                    matched = laneGraphTools.map_match_pose(pose, out_edge_index, out_edge_path_index);
                    //if vehicle was found on map, add vehicle to MultiVehicleTrajectoryPlanner
                    if(matched)
                    {
                        planner->set_vehicle(std::make_shared<VehicleTrajectoryPlanningState>(vehicle_id, out_edge_index, out_edge_path_index));
                        cpm::Logging::Instance().write(
                        3,
                        "Vehicle %d matched.",
                        int(vehicle_id)
                        );
                        /* -------------------------------------------------------------
                         * Finish initializing vehicle when we found its start position
                         * -------------------------------------------------------------
                         */
                        planner->set_writer(
                        std::make_shared<dds::pub::DataWriter<LaneGraphTrajectory>>(
                            writer_laneGraphTrajectory
                            )
                        );
                        planner->set_reader(
                        std::make_shared<dds::sub::DataReader<LaneGraphTrajectory>>(
                            reader_laneGraphTrajectory
                            )
                        );

                        //Start the Planner. That includes collision avoidance. In this case we avoid collisions by priority assignment
                        //with the consequence of speed reduction for the lower prioritized vehicle (here: Priority based on descending vehicle ID of the neighbours.)
                        planner->start();

                        auto command = planner->get_trajectory_command(t_now);
                        writer_vehicleCommandTrajectory.write(command);

                        break;
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
        }
    }
    }

    return 0;
}

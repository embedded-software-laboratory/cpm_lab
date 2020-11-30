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
#include "StopRequest.hpp"

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
    uint64_t dt_nanos = 400000000ull; // Needs to match period of planner
    // Definition of time stamp at which to stop
    const uint64_t trigger_stop = std::numeric_limits<uint64_t>::max();


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

    // Writer to send a StopRequest to LCC (in case of failure)
    dds::pub::DataWriter<StopRequest> writer_stopRequest(
            dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
            cpm::get_topic<StopRequest>("stopRequest")
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
    
    
    //FIXME: This should be replaced by a request to LCC
    // systemTrigger Reader, QoS Settings taken from QOS_READY_TRIGGER.xml
    dds::pub::DataWriter<SystemTrigger> writer_systemTrigger(
            dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
            cpm::get_topic<SystemTrigger>("systemTrigger")
    );

    /* ---------------------------------------------------------------------------------
     * Create planner object
     * ---------------------------------------------------------------------------------
     */
    /* Soll-Verhalten:
     *   - Wir starten erst, wenn eine StateList von der Middleware kommt
     *   - Wir schicken dann auch erst VehicleTrajectories an die Middleware, wenn eine
     *   weitere StateList kommt
     */
    auto planner = std::unique_ptr<VehicleTrajectoryPlanner>(new VehicleTrajectoryPlanner(dt_nanos));

    // Set reader/writers of planner so it can communicate with other planners
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

    bool received_stop = false;
    uint64_t t_now;
    while( !(received_stop || planner->is_crashed()) ) {

        dds::sub::LoanedSamples<VehicleStateList> state_samples = reader_vehicleStateList.take();
        for(auto sample : state_samples) {
            if( sample.info().valid() ) {

                // We received a StateList and need to send commands to vehicle now
                t_now = sample.data().t_now();

                if( sample.data().period_ms()*1e6 != dt_nanos ) {
                    cpm::Logging::Instance().write(
                            1,
                            "Please set middleware_period_ms to 400ms");
                    StopRequest request(vehicle_id);
                    writer_stopRequest.write(request);
                    return 1;
                }

                if(!planner->is_started())//will be set to true after fist activation
                {
                    cpm::Logging::Instance().write(3,
                        "Preparing to start planner");
                    // Set real time
                    planner->set_real_time(t_now);

                    /* ---------------------------------------------------------------------
                    * Check for start position of our vehicle
                    * ----------------------------------------------------------------------
                    */
                    bool matched = false;
                    //FIXME: This probably does not require a loop
                    for(auto vehicle_state : sample.data().state_list())
                    {
			std::cout << static_cast<uint32_t>(vehicle_state.vehicle_id()) << std::endl;
                        if( vehicle_id == vehicle_state.vehicle_id() ) {
                            auto pose = vehicle_state.pose();
                            int out_edge_index = -1;
                            int out_edge_path_index = -1;
                            matched = laneGraphTools.map_match_pose(pose, out_edge_index, out_edge_path_index);
                            if( !matched ) {
                                cpm::Logging::Instance().write(1,
                                    "Couldn't find starting position,\
                                    try moving the vehicle if this persists.");
                                StopRequest request(vehicle_id);
                                writer_stopRequest.write(request);
                            } else {

				    planner->set_vehicle(std::make_shared<VehicleTrajectoryPlanningState>(vehicle_id, out_edge_index, out_edge_path_index));
				    cpm::Logging::Instance().write(
				    3,
				    "Vehicle %d matched.",
				    int(vehicle_id)
				    );

				    //Start the Planner. That includes collision avoidance. In this case we avoid collisions by priority assignment
				    //with the consequence of speed reduction for the lower prioritized vehicle (here: Priority based on descending vehicle ID of the neighbours.)
				    planner->start();
			    }

                        }
                    }
			
		    if( !matched ) {
			cpm::Logging::Instance().write(1,
			    "Couldn't find vehicle in\
			    VehicleStateList.");
			StopRequest request(vehicle_id);
			writer_stopRequest.write(request);
		    }
                }

		if( planner->is_started() ) {
			// Set real time
			planner->set_real_time(t_now);
			//get trajectory commands from VehicleTrajectoryPlanner with new points
			auto command = planner->get_trajectory_command(t_now);

			writer_vehicleCommandTrajectory.write(command);
		}
            }
        }

        // Check if we received a SystemTrigger to stop
        dds::sub::LoanedSamples<SystemTrigger> systemTrigger_samples = reader_systemTrigger.take();
        for(auto sample : systemTrigger_samples) {
            if( sample.info().valid() && 
                   (sample.data().next_start().nanoseconds() == trigger_stop)
              ) {
            cpm::Logging::Instance().write(
				2,
				"Received stop signal, stopping"
				);
                received_stop = true;
                TimeStamp timestamp(trigger_stop);
                SystemTrigger stop_trigger(timestamp);
                writer_systemTrigger.write(stop_trigger);
            }
        }
    }

    return 0;
}

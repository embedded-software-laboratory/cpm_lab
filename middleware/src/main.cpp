/**
 * \class main.cpp
 * \brief This class includes the main function and is mainly responsible for the initialization of the middleware
 */

#include <memory>
#include <sstream>
#include <string>
#include <functional>
#include <dds/core/QosProvider.hpp>

#include <dds/sub/ddssub.hpp>

#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"

#include "VehicleStateList.hpp"

#include "Communication.hpp"

int main (int argc, char *argv[]) { 
    //Initialize the cpm logger, set domain id etc
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("middleware_main"); 

    //Initial parameters that can partially be set in the command line
    //Timer parameters
    std::string node_id = cpm::cmd_parameter_string("node_id", "middleware", argc, argv);
    uint64_t period_nanoseconds = cpm::cmd_parameter_uint64_t("period_nanoseconds", 250000000, argc, argv);
    uint64_t offset_nanoseconds = cpm::cmd_parameter_uint64_t("offset_nanoseconds", 1, argc, argv);
    bool simulated_time_allowed = true;
    bool simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    bool wait_for_start = cpm::cmd_parameter_bool("wait_for_start", true, argc, argv);

    //Communication parameters
    int hlcDomainNumber = cpm::cmd_parameter_int("domain_number", 1, argc, argv); 
    
    //Vehicle ID(s) set in command line, correspond to HLC IDs
    //Vehicle amount: Tell system amount of vehicles, IDs range from 1 to vehicle_amount
    int vehicleID = cpm::cmd_parameter_int("vehicle_id", 1, argc, argv); 
    int amount_of_vehicles = cpm::cmd_parameter_int("vehicle_amount", -1, argc, argv);
    std::vector<int> default_ids{ 1 };
    std::vector<int> vehicle_ids = cpm::cmd_parameter_ints("vehicle_ids", default_ids, argc, argv);

    //Constants - topic names
    const std::string logTopicName = "log";
    const std::string vehicleStateListTopicName = "vehicleStateList"; 
    const std::string vehicleTrajectoryTopicName = "vehicleCommandTrajectory";
    const std::string vehicleSpeedCurvatureTopicName = "vehicleCommandSpeedCurvature"; 
    const std::string vehicleDirectTopicName = "vehicleCommandDirect"; 

    std::cout << "DEBUG: - configuration" << std::endl
        << "Node ID: " << node_id
        << "Domain ID: " << cpm::cmd_parameter_int("dds_domain", 0, argc, argv)
        << "Simulated time: " << simulated_time
        << "Wait for start: " << wait_for_start;

    //Get unsigned vehicle ids only if vehicle_amount was not correctly set
    std::vector<uint8_t> unsigned_vehicle_ids;
    if (amount_of_vehicles > 0 && amount_of_vehicles <= 255) {
        //Get vehicle ID list from set vehicle amount
        uint8_t u_amount_of_vehicles = static_cast<uint8_t>(amount_of_vehicles);
        for (uint8_t vehicle_id = 1; vehicle_id <= u_amount_of_vehicles; ++vehicle_id) {
            unsigned_vehicle_ids.push_back(static_cast<uint8_t>(vehicle_id));
        }
    }
    else {
        //Create vehicle ID list from command line id list 
        for (int vehicle_id : vehicle_ids) {
            if (vehicle_id >= 0 && vehicle_id <= 255) {
                unsigned_vehicle_ids.push_back(static_cast<uint8_t>(vehicle_id));
            }
            else {
                std::cerr << "Incompatible vehicle id" << std::endl;
                cpm::Logging::Instance().write("Incompatible vehicle ids sent - not within 0 and 255");
            }
        }
        if (unsigned_vehicle_ids.size() == 0) {
            std::cerr << "No vehicle ids set!" << std::endl; //TODO LOG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            exit(EXIT_FAILURE);
        }
    }

    //Initialize the timer
    std::cout << "Initializing Timer..." << std::endl;
    std::shared_ptr<cpm::Timer> timer = cpm::Timer::create(node_id, period_nanoseconds, offset_nanoseconds, wait_for_start, simulated_time_allowed, simulated_time);
    std::cout << "...done." << std::endl;

    //Initialize the communication (TODO later: depending on message type for commands, can change dynamically)
    std::cout << "Initializing Communication..." << std::endl;
    std::shared_ptr<Communication> communication = std::make_shared<Communication>(
        hlcDomainNumber,
        vehicleStateListTopicName,
        vehicleTrajectoryTopicName,
        vehicleSpeedCurvatureTopicName,
        vehicleDirectTopicName,
        vehicleID,
        timer,
        unsigned_vehicle_ids
    );
    std::cout << "...done." << std::endl;

    //Wait for HLC program to send ready signal
    std::cout << "Waiting for HLC..." << std::endl;
    communication->wait_for_hlc_ready_msg(unsigned_vehicle_ids);
    std::cout << "...done." << std::endl;

    //Wait for start signal (done by the timer after start)
    //Start the communication with the HLC
    using namespace std::placeholders;
    timer->start_async([&](uint64_t t_now) {
        std::vector<VehicleState> states = communication->getLatestVehicleMessages(t_now);
        std::vector<VehicleObservation> observations = communication->getLatestVehicleObservationMessages(t_now);

        //Transform to VehicleStateList message
        rti::core::vector<VehicleState> rti_states(states);
        rti::core::vector<VehicleObservation> rti_observations(observations);
        VehicleStateList state_list;
        state_list.state_list(rti_states);
        state_list.vehicle_observation_list(rti_observations);
        state_list.t_now(t_now);

        //Send newest vehicle state list to the HLC
        communication->sendToHLC(state_list);

        //Log the received vehicle data size / sample size for verbose log level
        std::stringstream stream;
        stream << "Got latest messages, state array size: " << states.size();
        if (states.size() > 0) {
            stream << " - sample data: " << states.at(0).battery_voltage();
        }
        cpm::Logging::Instance().write(3, stream.str().c_str());

        //Get the last response time of the HLC
        // Real time -> Print an error message if a period has been missed
        // Simulated time -> Busy waiting until an answer for all connected HLCs (vehicle_ids) has been received
        std::map<uint8_t, uint64_t> lastHLCResponseTimes;
        lastHLCResponseTimes = communication->getLastHLCResponseTimes();

        if (simulated_time) {
            //Wait until any command has been received for all vehicle ids
            bool id_missing = true;
            while(id_missing) {
                id_missing = false;
                for (uint8_t id : unsigned_vehicle_ids) {
                    if (lastHLCResponseTimes.find(id) == lastHLCResponseTimes.end()) {
                        id_missing = true;
                        break;
                    }
                }

                usleep(100000);
                lastHLCResponseTimes = communication->getLastHLCResponseTimes();
            }

            //Check if the last message was received in the current timestep
            bool message_missing = true;
            while (message_missing) {
                message_missing = false;
                for (std::map<uint8_t, uint64_t>::iterator it = lastHLCResponseTimes.begin(); it != lastHLCResponseTimes.end(); ++it){
                    if (it->second != timer->get_time()) {
                        message_missing = true;
                        //break; -> Program does not work for some reason if break is used at this point
                    }
                }

                usleep(100000);
                lastHLCResponseTimes = communication->getLastHLCResponseTimes();
            }
        }
        else {
            //Log error for each HLC if a time step was missed
            //Method:   Check last time step that was logged when a message from the HLC was received in TypedCommunication
            //          (If there is none, see below - an error is logged then that no message was ever received)
            //          Compare this received_time with the current period_time t_now - if no message was received in the last timestep,
            //          then the time between both timestamps is always greater than period_nanoseconds (one period) -> an error can be logged
            //          This does NOT work if the message is received in between starting the next period and fetching the last response times
            //          But: The time discrepancy should be so small that this behaviour is not considered problematic
            for (std::map<uint8_t, uint64_t>::iterator it = lastHLCResponseTimes.begin(); it != lastHLCResponseTimes.end(); ++it) {
                uint64_t passed_time = (t_now - it->second);
                if (passed_time > period_nanoseconds) {
                    std::stringstream stream;
                    stream << "Timestep missed by HLC number " << static_cast<uint32_t>(it->first) << ", last response: " << (it->second) 
                        << ", current time: " << t_now 
                        << ", periods missed: " << passed_time / period_nanoseconds;
                    cpm::Logging::Instance().write(stream.str().c_str());
                }

                //Evaluation log - only for higher log level
                // cpm::Logging::Instance().write(
                //     "Vehicle: %u, Received HLC timestamp: %llu, Valid after timestamp: %llu", 
                //     it->first, 
                //     it->second, 
                //     t_now
                // );
            }
            //Also log an error if no HLC data has yet been received
            for (uint8_t id : unsigned_vehicle_ids) {
                if (lastHLCResponseTimes.find(id) == lastHLCResponseTimes.end()) {
                    std::stringstream stream;
                    stream << "HLC number " << static_cast<uint32_t>(id) << " has not yet sent any data";
                    cpm::Logging::Instance().write(stream.str().c_str());
                }
            }
        }
    });

    std::cout << "Program started" << std::endl;

    //Stop the program when enter is pressed
    std::cin.get();
    std::cout << "Exiting program" << std::endl;
    timer->stop();
}
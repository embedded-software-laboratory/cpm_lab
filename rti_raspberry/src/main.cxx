#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <string>
#include <iterator>
#include <iomanip>
#include <ios>
using std::vector;

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/util/util.hpp> // for sleep()

#include "VehicleCommandDirect.hpp"
#include "VehicleCommandSpeedCurvature.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "VehicleState.hpp"
#include "cpm/Timer.hpp"

#include "SensorCalibration.hpp"
#include "Localization.hpp"
#include "Controller.hpp"


#ifdef VEHICLE_SIMULATION
#include "SimulationVehicle.hpp"
#include "SimulationIPS.hpp"
#endif

#include "bcm2835.h"

extern "C" {
#include "spi.h"
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h"
}


// TODO move to cpm_lib
uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

bool check_CRC_miso(spi_miso_data_t spi_miso_data) { 
    uint16_t mosi_CRC = spi_miso_data.CRC;
    spi_miso_data.CRC = 0;
    return mosi_CRC == crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));
}


// TODO move to cpm_lib
template<typename T>
struct VehicleIDFilteredTopic : public dds::topic::ContentFilteredTopic<T>
{
    VehicleIDFilteredTopic(const dds::domain::DomainParticipant &participant, const std::string &topic_name, const uint8_t &vehicle_id)
    :dds::topic::ContentFilteredTopic<T>(
        dds::topic::Topic<T>(participant, topic_name), 
        topic_name + "_vehicle_id_filtered", 
        dds::topic::Filter("vehicle_id = " + std::to_string(vehicle_id))
    )
    {
        static_assert(std::is_same<decltype(std::declval<T>().vehicle_id()), uint8_t>::value, "IDL type must have a vehicle_id.");
    }
};



// TODO move to cpm_lib
#include <type_traits>
template<typename T>
bool take_new_sample(dds::sub::DataReader<T> &reader, T& value_out, uint64_t t_now)
{
    static_assert(std::is_same<decltype(value_out.header()), Header&>::value, "IDL type must have a header.");

    bool has_new_sample = false;    

    // TODO is there a type-safe alternative for the Query?
    // TODO test this!
    dds::sub::LoanedSamples<T> new_samples = reader.select().content(dds::sub::Query(reader, "header.valid_after_stamp.nanoseconds <= " + std::to_string(t_now))).take();

    for(auto sample : new_samples)
    {
        if(sample.info().valid())
        {
            // TODO if there are multiple valid samples, use the one with the latest header.created_stamp
            value_out = sample.data();
            has_new_sample = true;
        }
    }

    return has_new_sample;
}

int main(int argc, char *argv[])
{

    if(argc != 2) {
        std::cerr << "Usage: vehicle_rpi_firmware <vehicle-id>" << std::endl;
        return 1;
    }

    const int vehicle_id = std::atoi(argv[1]);

    if(vehicle_id < 0 || vehicle_id >= 255) {
        std::cerr << "Invalid vehicle ID." << std::endl;
        return 1;
    }
    std::cout << "vehicle_id " << vehicle_id << std::endl;


#ifndef VEHICLE_SIMULATION
    // Hardware setup
    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }
    spi_init();
#else
    SimulationVehicle simulationVehicle;
    SimulationIPS simulationIPS;
#endif

    crcInit();

    // DDS setup
    dds::domain::DomainParticipant participant (0);

    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    dds::pub::DataWriter<VehicleState> writer_vehicleState(dds::pub::Publisher(participant), topic_vehicleState, QoS);


    VehicleIDFilteredTopic<VehicleCommandDirect> topic_vehicleCommandDirect(participant, "vehicleCommandDirect", vehicle_id);
    dds::sub::DataReader<VehicleCommandDirect> reader_vehicleCommandDirect(dds::sub::Subscriber(participant), topic_vehicleCommandDirect);

    VehicleIDFilteredTopic<VehicleCommandSpeedCurvature> topic_vehicleCommandSpeedCurvature(participant, "vehicleCommandSpeedCurvature", vehicle_id);
    dds::sub::DataReader<VehicleCommandSpeedCurvature> reader_vehicleCommandSpeedCurvature(dds::sub::Subscriber(participant), topic_vehicleCommandSpeedCurvature);

    VehicleIDFilteredTopic<VehicleCommandTrajectory> topic_vehicleCommandTrajectory(participant, "vehicleCommandTrajectory", vehicle_id);
    dds::sub::DataReader<VehicleCommandTrajectory> reader_vehicleCommandTrajectory(dds::sub::Subscriber(participant), topic_vehicleCommandTrajectory);


    // Control loop
    int latest_command_TTL = 0;
    Localization localization;
    Controller controller;
    int loop_counter = 0;


    const uint64_t period_nanoseconds = 20000000ull; // 50 Hz

    auto update_loop = cpm::Timer::create("Raspberry_" + std::to_string(vehicle_id), period_nanoseconds, 0);


    update_loop->start([&](uint64_t t_iteration_start){

        try 
        {
            // Read new commands, reset watchdog countdown on new command
            {
                VehicleCommandDirect sample_CommandDirect;
                if(take_new_sample(reader_vehicleCommandDirect, sample_CommandDirect, t_iteration_start))
                {
                    controller.update_command(sample_CommandDirect);
                    latest_command_TTL = 25;
                }
            }
            {
                VehicleCommandSpeedCurvature sample_CommandSpeedCurvature;
                if(take_new_sample(reader_vehicleCommandSpeedCurvature, sample_CommandSpeedCurvature, t_iteration_start))
                {
                    controller.update_command(sample_CommandSpeedCurvature);
                    latest_command_TTL = 25;
                }
            }
            {
                VehicleCommandTrajectory sample_CommandTrajectory;
                if(take_new_sample(reader_vehicleCommandTrajectory, sample_CommandTrajectory, t_iteration_start))
                {
                    controller.update_command(sample_CommandTrajectory);
                    latest_command_TTL = 25;
                }
            }


            // Watchdog countdown for loss of signal
            if(latest_command_TTL > 0) {
                latest_command_TTL--;
            }
            else {
                controller.vehicle_emergency_stop();
            }

            // Run controller
            spi_mosi_data_t spi_mosi_data = controller.get_control_signals(t_iteration_start);
            spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));


#ifdef VEHICLE_SIMULATION
            spi_miso_data_t spi_miso_data = simulationVehicle.update(spi_mosi_data, period_nanoseconds/1e9);
#else
            // Exchange data with low level micro-controller
            spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
#endif            
     

            // Process sensor data
            if(check_CRC_miso(spi_miso_data)) {
                VehicleState vehicleState = SensorCalibration::convert(spi_miso_data);
                Pose2D new_pose = localization.sensor_update(vehicleState);
                vehicleState.pose(new_pose);
                vehicleState.header().create_stamp().nanoseconds(t_iteration_start);
                vehicleState.vehicle_id(vehicle_id);

                controller.update_vehicle_state(vehicleState);
                writer_vehicleState.write(vehicleState);
            }
            else {
                std::cerr << "[" << std::fixed << std::setprecision(9) << double(clock_gettime_nanoseconds())/1e9 <<  "] Data corruption on ATmega SPI bus. CRC mismatch." << std::endl;
            }

            if(loop_counter == 10) {
                localization.reset(); // ignore initial signals
            }

            loop_counter++;
        }
        catch(const dds::core::Exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    });
    
    return 0;
}

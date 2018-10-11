#include <iostream>

#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> // for sleep()

#include "VehicleState.hpp"
#include "AbsoluteTimer.hpp"

int main(int argc, char *argv[])
{
    dds::domain::DomainParticipant participant (0);
    dds::topic::Topic<VehicleState> topic (participant, "vehicleState");

    auto QoS = dds::pub::qos::DataWriterQos();

    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);


    dds::pub::DataWriter<VehicleState> writer(dds::pub::Publisher(participant), topic, QoS);

    int count = 0;

    AbsoluteTimer timer_loop(0, 500000000, 0, 0, [&](){

        VehicleState sample;
        sample.odometer_distance(count);

        std::cout << "Writing VehicleState, count " << count << std::endl;
        writer.write(sample);

        count++;

    });
    

    while(1) sleep(1);
    return 0;
}

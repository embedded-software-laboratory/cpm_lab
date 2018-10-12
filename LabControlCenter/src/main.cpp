#include "defaults.hpp"
#include "Joystick.hpp"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include "VehicleCommand.hpp"
#include "AbsoluteTimer.hpp"

int main(/*int argc, char *argv[]*/)
{
    auto joystick = make_shared<Joystick>("/dev/input/js1");


    // DDS start
    dds::domain::DomainParticipant participant (0);
    dds::topic::Topic<VehicleCommand> topic (participant, "vehicleCommand");
    auto QoS = dds::pub::qos::DataWriterQos();
    auto reliability = dds::core::policy::Reliability::BestEffort();
    reliability.max_blocking_time(dds::core::Duration(0,0));
    QoS.policy(reliability);
    dds::pub::DataWriter<VehicleCommand> writer(dds::pub::Publisher(participant), topic, QoS);


    AbsoluteTimer timer_loop(0, 20000000, 0, 0, [&](){
        cout << joystick->getAxis(2) << endl;
        VehicleCommand sample;
        sample.vehicle_id(0);
        sample.motor_throttle(joystick->getAxis(1) / (-double(1<<15)));
        sample.steering_angle(joystick->getAxis(2) / (-double(1<<15)));
        writer.write(sample);

    });

    while(1) sleep(1);
    return 0;
}
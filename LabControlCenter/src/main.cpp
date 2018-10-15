#include "defaults.hpp"
#include "stdio.h"
#include "Joystick.hpp"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "VehicleCommand.hpp"
#include "VehicleState.hpp"
#include "AbsoluteTimer.hpp"


uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

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


    dds::topic::Topic<VehicleState> topic_vehicleState (participant, "vehicleState");
    dds::sub::DataReader<VehicleState> reader_vehicleState(dds::sub::Subscriber(participant), topic_vehicleState);




    AbsoluteTimer timer_loop(0, 20000000, 0, 0, [&](){


        VehicleCommand sample;
        sample.vehicle_id(0);
        sample.data()._d(VehicleCommandMode_def::DirectControlMode);
        sample.data().direct_control().motor_throttle(joystick->getAxis(1) / (-double(1<<15)));
        sample.data().direct_control().steering_servo(joystick->getAxis(2) / (-double(1<<15)));

        writer.write(sample);


        printf("motor_throttle %12.4f  steering_servo %12.4f\n", 
            sample.data().direct_control().motor_throttle(), 
            sample.data().direct_control().steering_servo());




        // Read new states
        {
            vector<dds::sub::Sample<VehicleState>> new_states;
            if(reader_vehicleState.take(std::back_inserter(new_states)) > 0) 
            {
                uint64_t stamp = new_states.back().data().stamp().nanoseconds();
                uint64_t t_local = clock_gettime_nanoseconds();
                printf("local %20lu   stamp %20lu  delta %20lu \n", 
                    clock_gettime_nanoseconds(), 
                    stamp,
                    (int64_t(t_local) - int64_t(stamp)) );
            }
        }



    });

    while(1) sleep(1);
    return 0;
}
#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include "cpm/AsyncReader.hpp"
#include "LedPoints.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "types.hpp"


void process_LedPoints(LedPoints led_points)
{
    std::cout << "recvd " << led_points.led_points().size() << " LEDs" << std::endl;

    VehiclePointTimeseries asd;
}


int main(int argc, char* argv[])
{
    cpm::AsyncReader<LedPoints> ipsLedPoints_reader(
        [](dds::sub::LoanedSamples<LedPoints>& samples){
            for(auto sample : samples) 
                if(sample.info().valid()) 
                    process_LedPoints(sample.data());
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<LedPoints>("ipsLedPoints")
    );

    // start replay
    system("rtireplay -cfgName mydefault -cfgFile recordings/replay_config.xml");    
    return 0;
}
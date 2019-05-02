#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include "cpm/AsyncReader.hpp"
#include "LedPoints.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "IpsPipeline.hpp"



int main(int argc, char* argv[])
{
    IpsPipeline ipsPipeline;


    cpm::AsyncReader<LedPoints> ipsLedPoints_reader(
        [&](dds::sub::LoanedSamples<LedPoints>& samples){
            for(auto sample : samples) 
                if(sample.info().valid()) 
                    ipsPipeline.apply(sample.data());
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<LedPoints>("ipsLedPoints")
    );

    // stop previous replay
    system("killall rtireplay");

    // start replay
    system("rtireplay -cfgName mydefault -cfgFile recordings/replay_config.xml");
    exit(0);
    return 0;
}
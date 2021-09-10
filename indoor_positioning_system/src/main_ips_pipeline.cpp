#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>

#include "cpm/AsyncReader.hpp"
#include "LedPoints.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"
#include "cpm/init.hpp"
#include "IpsPipeline.hpp"


/**
 * \brief This process receives the LED points provided in \link main_led_detection.cpp \endlink
 * and uses them to detect the vehicles, their IDs, and their positions.
 * \ingroup ips
 */
int main(int argc, char* argv[])
{
    if(argc < 2) {
        std::cout << "To enable visualization use parameter --visualization=1" << std::endl;
    }

    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("ips_pipeline");

    const bool enable_visualization = cpm::cmd_parameter_bool("visualization", false, argc, argv);
    IpsPipeline ipsPipeline(enable_visualization);


    cpm::AsyncReader<LedPoints> ipsLedPoints_reader(
        [&](std::vector<LedPoints>& samples){
            for(auto& data : samples) 
                ipsPipeline.apply(data);
        }, 
        "ipsLedPoints"
    );

    if(argc > 1 && std::string(argv[1]) == "replay")
    {
        // stop previous replay
        system("killall -9 rtireplay");

        // start replay
        system("rtireplay -cfgName mydefault -cfgFile recordings/replay_config.xml");
    }
    else
    {
        while(1) sleep(1);
    }

    exit(0);
    return 0;
}
/**
 * \class main.cpp
 * \brief This file includes a reader that receives NUC messages
 */

#include <memory>
#include <sstream>
#include <string>
#include <functional>

#include <dds/pub/ddspub.hpp>

#include "ReadyStatus.hpp"

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/AsyncReader.hpp"

int main (int argc, char *argv[]) { 
    //Initialize the cpm logger, set domain id etc
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("hlc_hello");

    //Create DataReader that reads NUC ready messages
    cpm::AsyncReader<ReadyStatus> reader(
        [](dds::sub::LoanedSamples<ReadyStatus>& samples){
            for (auto sample : samples)
            {
                if(sample.info().valid())
                {
                    auto data = sample.data();
                    std::cout << "Received: " << data << std::endl;
                }
            }
        },
        cpm::ParticipantSingleton::Instance(),
        cpm::get_topic<ReadyStatus>("hlc_startup")
    );

    std::cout << "Press Enter to stop the program" << std::endl;
    std::cin.get();

    return 0;
}
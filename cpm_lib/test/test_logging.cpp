#include "catch.hpp"
#include "cpm/Logging.hpp"
#include <unistd.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <thread>

#include <dds/domain/DomainParticipant.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>

#include "ParameterRequest.hpp"

#include "cpm/ParticipantSingleton.hpp"

TEST_CASE( "Logging" ) {
    //Make sure that the Logging topic already exists
    Logging::Instance();

    //Create logging reader
    dds::sub::DataReader<ParameterRequest> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), dds::topic::find<dds::topic::Topic<ParameterRequest>>(cpm::ParticipantSingleton::Instance(), "Logs"), (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));
    // Create a WaitSet
    dds::core::cond::WaitSet waitset;
    // Create a ReadCondition for a reader with a specific DataState
    dds::sub::cond::ReadCondition read_cond(
        reader, dds::sub::status::DataState::any());
    // Attach conditions
    waitset += read_cond;

    //Get Stringstream version
    std::stringstream actual_content;
    actual_content << "TEST";

    //Thread for testing whether the logs are sent correctly via DDS
    std::thread signal_thread = std::thread([&](){
        waitset.wait();
        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                CHECK(sample.data().name() == actual_content.str());
            }
        }
    });

    Logging::Instance() << "TEST";
    Logging::Instance().flush();

    //Check file content
    std::ifstream file;
    std::string str;
    std::stringstream file_content;
    file.open("Log.txt");
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str;
    }
	file.close();

    //Compare file content with desired content
    CHECK(file_content.str() == actual_content.str());

    signal_thread.join();
}
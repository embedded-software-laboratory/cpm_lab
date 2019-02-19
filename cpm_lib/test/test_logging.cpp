#include "catch.hpp"
#include "Logging.hpp"
#include <unistd.h>

#include <sstream>
#include <iostream>
#include <fstream>

#include <dds/domain/DomainParticipant.hpp>
#include <dds/sub/ddssub.hpp>

#include "ParameterRequest.hpp"

#include "cpm/ParticipantSingleton.hpp"

TEST_CASE( "Logging" ) {
    Logging::Instance();
    //Create logging reader
    dds::topic::Topic<ParameterRequest> loggingTopic(cpm::ParticipantSingleton::Instance(), "Logs");
    dds::sub::DataReader reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), loggingTopic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));

    Logging::Instance() << "TEST";

    //Get Stringstream version
    std::stringstream actual_content;
    actual_content << "TEST";

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
}
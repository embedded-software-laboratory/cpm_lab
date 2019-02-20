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

#include "Log.hpp"

#include "cpm/ParticipantSingleton.hpp"

TEST_CASE( "Logging" ) {
    //Make sure that the Logging topic already exists
    Logging::Instance();

    std::string id = "TestID";
    Logging::Instance().set_id(id);

    //Create logging reader
    dds::sub::DataReader<Log> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), dds::topic::find<dds::topic::Topic<Log>>(cpm::ParticipantSingleton::Instance(), "Logs"), (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));
    // Create a WaitSet
    dds::core::cond::WaitSet waitset;
    // Create a ReadCondition for a reader with a specific DataState
    dds::sub::cond::ReadCondition read_cond(
        reader, dds::sub::status::DataState::any());
    // Attach conditions
    waitset += read_cond;

    //Get Stringstream version to check if Logging like a stringstream (which it should)
    std::stringstream actual_content;
    actual_content << "TEST";

    std::string second_test = "Second test!";
    std::string with_more = "With more!";

    //Thread for testing whether the logs are sent correctly via DDS
    std::thread signal_thread = std::thread([&](){
        waitset.wait();
        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                CHECK(sample.data().content() == actual_content.str());
                CHECK(sample.data().id() == id);
            }
        }
        waitset.wait();
        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                CHECK(sample.data().content() == second_test + with_more);
            }
        }
    });

    Logging::Instance() << "TEST" << std::endl;

    //Check file content
    std::ifstream file;
    std::string str;
    std::stringstream file_content;
    file.open("Log.csv");
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str;
    }
	file.close();

    //Compare file content with desired content
    CHECK(file_content.str().find(actual_content.str()) != std::string::npos);

    //Some milliseconds need to pass, else order is not guaranteed
    rti::util::sleep(dds::core::Duration::from_millisecs(100));

    Logging::Instance() << second_test << with_more << std::endl;

    //Check file content
    str.clear();
    file_content.str(std::string());
    file.open("Log.csv");
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str << "\n";
    }
	file.close();

    //Compare file content with desired content
    CHECK(file_content.str().find(actual_content.str()) != std::string::npos);
    CHECK(file_content.str().find(second_test + with_more) != std::string::npos);

    signal_thread.join();
}
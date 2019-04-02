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

    //Create logging logs_reader
    dds::sub::DataReader<Log> logs_reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), dds::topic::find<dds::topic::Topic<Log>>(cpm::ParticipantSingleton::Instance(), "Logs"), (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));

    // Create a WaitSet that waits for any data
    dds::core::cond::WaitSet waitset;
    dds::sub::cond::ReadCondition read_cond(
        logs_reader, dds::sub::status::DataState::any());
    waitset += read_cond;

    //Get Stringstream version to check if the Logger treats data like a stringstream (which it should)
    std::stringstream actual_content;
    std::string first_test = "TEST";
    std::string second_test = "Second test!";
    std::string with_more = "With more!";

    //Data from the threads for later checks - CHECK does not support concurrency
    std::string thread_content_1;
    std::string thread_id;
    std::string thread_content_2;

    //Thread for testing whether the logs are sent correctly via DDS
    //The thread waits for the log signals and stores them in thread_content_1 and thread_content_2 for later checks
    std::thread signal_thread = std::thread([&](){
        waitset.wait();
        for (auto sample : logs_reader.take()) {
            if (sample.info().valid()) {
                thread_content_1 = sample.data().content();
                thread_id = sample.data().id();
            }
        }
        waitset.wait();
        for (auto sample : logs_reader.take()) {
            if (sample.info().valid()) {
                thread_content_2 = sample.data().content();
            }
        }
    });

    //Log first test data and write it to the stringstream for comparison
    actual_content << first_test;
    Logging::Instance().write(first_test.c_str());

    //Store the current file content of the log file - it should match the actual_content stringstream
    usleep(100000); //Sleep 100ms to let the Logger access the file first
    std::ifstream file;
    std::string str;
    std::stringstream file_content;
    file.open(Logging::Instance().get_filename());
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str;
    }
	file.close();

    //Compare file content with desired content
    CHECK(file_content.str().find(actual_content.str()) != std::string::npos);

    //Some milliseconds need to pass, else the order of the logs is not guaranteed
    rti::util::sleep(dds::core::Duration::from_millisecs(250));

    //Write second message to Logger
    std::stringstream stream;
    stream << second_test << with_more;
    Logging::Instance().write(stream.str().c_str());
    stream.clear();

    //Write C-style message
    Logging::Instance().write("Die Zahl %i nennt sich auch %s", 5, "fünf");

    //Store the (now changed) file content of the log file
    usleep(10000); //Sleep 10ms to let the Logger access the file first
    str.clear();
    file_content.str(std::string());
    file.open(Logging::Instance().get_filename());
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str << "\n";
    }
	file.close();

    //Compare file content with desired content
    CHECK(file_content.str().find(actual_content.str()) != std::string::npos);
    CHECK(file_content.str().find(second_test + with_more) != std::string::npos);
    CHECK(file_content.str().find("Die Zahl 5 nennt sich auch fünf") != std::string::npos);

    signal_thread.join();

    //Compare thread content (received messages) with desired content
    CHECK(thread_content_1 == actual_content.str());
    CHECK(thread_id == id);
    CHECK(thread_content_2 == second_test + with_more);
}
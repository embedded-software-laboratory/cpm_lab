#include "catch.hpp"
#include "cpm/Logging.hpp"
#include <unistd.h>

#include <algorithm>
#include <sstream>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <chrono>

#include "Log.hpp"

#include "cpm/ReaderAbstract.hpp"
#include "cpm/ParticipantSingleton.hpp"

/**
 * \test Tests Logging
 * \ingroup cpmlib
 */
TEST_CASE( "Logging" ) {
    //Make sure that the Logging topic already exists
    cpm::Logging::Instance();

    std::string id = "TestID";
    cpm::Logging::Instance().set_id(id);

    //Create logging logs_reader
    cpm::ReaderAbstract<Log> logs_reader("log", true, true);

    //It usually takes some time for all instances to see each other - wait until then
    std::cout << "Waiting for DDS entity match in Logging test" << std::endl << "\t";
    bool wait = true;
    while (wait)
    {
        usleep(10000); //Wait 10ms
        std::cout << "." << std::flush;

        if (logs_reader.matched_publications_size() > 0)
            wait = false;
    }
    std::cout << std::endl;

    //Get Stringstream version to check if the Logger treats data like a stringstream (which it should)
    std::stringstream actual_content;
    std::string first_test = "TEST";
    std::string second_test = "Second\" test!";
    std::string with_more = "With \"m\"ore!";
    std::string third_test = "Die Zahl 5 nennt sich auch fünf";

    //Log first test data and write it to the stringstream for comparison
    actual_content << first_test;
    cpm::Logging::Instance().write("%s", first_test.c_str());

    //Store the current file content of the log file - it should match the actual_content stringstream
    usleep(100000); //Sleep 100ms to let the Logger access the file first
    std::ifstream file;
    std::string str;
    std::stringstream file_content;
    file.open(cpm::Logging::Instance().get_filename());
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str;
    }
	file.close();

    //Compare file content with desired content
    CHECK(file_content.str().find(actual_content.str()) != std::string::npos);

    //Some milliseconds need to pass, else the order of the logs is not guaranteed
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    //Write second message to Logger
    std::stringstream stream;
    stream << second_test << with_more;
    cpm::Logging::Instance().write("%s", stream.str().c_str());
    stream.clear();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    //Write C-style message
    cpm::Logging::Instance().write("Die Zahl %i nennt sich auch %s", 5, "fünf");

    //Store the (now changed) file content of the log file
    usleep(10000); //Sleep 10ms to let the Logger access the file first
    str.clear();
    file_content.str(std::string());
    file.open(cpm::Logging::Instance().get_filename());
    while (std::getline(file, str)) {
        //String to second stringstream
        file_content << str << "\n";
    }
	file.close();

    //Compare file content with desired content
    std::string second_test_escaped = std::string(second_test);
    second_test_escaped += with_more;
    std::string escaped_quote = std::string("\"\"");
    size_t pos = 0;
    while ((pos = second_test_escaped.find('"', pos)) != std::string::npos) {
        second_test_escaped.replace(pos, 1, escaped_quote);
        pos += escaped_quote.size();
    }
    //Also put the whole string in quotes
    second_test_escaped.insert(0, "\"");
    second_test_escaped += "\"";

    CHECK(file_content.str().find(actual_content.str()) != std::string::npos);
    CHECK(file_content.str().find(second_test_escaped) != std::string::npos);
    CHECK(file_content.str().find(third_test) != std::string::npos);

    //Get listener data to check if it logs were received via DDS
    //Allow for some more waiting in between, in case the network is slow during testing
    //Here: Up to 1 second or until all data has been received
    std::vector<std::string> listener_content;
    std::string thread_id;
    for (int i = 0; i < 10; ++i)
    {
        for (auto& data : logs_reader.take()) {
            listener_content.push_back(data.content());
            thread_id = data.id();
        }

        if (listener_content.size() < 3)
        {
            usleep(100000);
        }
        else break;
    }

    //Compare thread content (received messages) with desired content (order irrelevant)
    CHECK((std::find(listener_content.begin(), listener_content.end(), actual_content.str()) != listener_content.end() &&
        std::find(listener_content.begin(), listener_content.end(), (second_test + with_more)) != listener_content.end() &&
        std::find(listener_content.begin(), listener_content.end(), third_test) != listener_content.end()));
    CHECK(thread_id == id);
}
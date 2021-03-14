// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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

    //Sleep 100ms to make sure that the logger reader is ready to receive messages
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
    std::vector<std::string> listener_content;
    std::string thread_id;
    for (auto& data : logs_reader.take()) {
        listener_content.push_back(data.content());
        thread_id = data.id();
    }

    //Compare thread content (received messages) with desired content (order irrelevant)
    CHECK((std::find(listener_content.begin(), listener_content.end(), actual_content.str()) != listener_content.end() &&
        std::find(listener_content.begin(), listener_content.end(), (second_test + with_more)) != listener_content.end() &&
        std::find(listener_content.begin(), listener_content.end(), third_test) != listener_content.end()));
    CHECK(thread_id == id);
}
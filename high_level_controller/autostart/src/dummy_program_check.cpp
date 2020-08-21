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

/**
 * \class main.cpp
 * \brief This is a test program to check if HLCs reply to the program check msg
 */

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <functional>

#include <dds/pub/ddspub.hpp>

#include "ReadyStatus.hpp"
#include "RemoteProgramCheck.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/TimerFD.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"

//To get the IP address
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <stdio.h>

//To spawn a process & get its PID
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

int main (int argc, char *argv[]) { 
    //Initialize the cpm logger, set domain id etc
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("dummy_check"); 

    //Create reader and writer to check HLC answers
    dds::topic::Topic<RemoteProgramCheck> program_check_topic = cpm::get_topic<RemoteProgramCheck>(cpm::ParticipantSingleton::Instance(), "remote_program_check");
    dds::pub::DataWriter<RemoteProgramCheck> program_check_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), program_check_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll()));

    cpm::AsyncReader<RemoteProgramCheck> program_check_reader(
        [&](dds::sub::LoanedSamples<RemoteProgramCheck>& samples){
            for(auto sample : samples)
            {
                if(sample.info().valid())
                {
                    //Only print answers
                    if ((sample.data().is_answer()))
                    {
                        std::cout << "Received answer: \n"
                            << "\tSource ID: " << sample.data().source_id()
                            << "\tScript running: " << sample.data().script_running()
                            << "\tMiddleware running: " << sample.data().middleware_running()
                            << std::endl;
                    }
                }
            }
        },
        cpm::ParticipantSingleton::Instance(),
        program_check_topic,
        true,
        false
    );

    //Send program check request
    RemoteProgramCheck check_msg;
    check_msg.source_id("dummy_check");
    check_msg.is_answer(false);
    check_msg.count(0);
    program_check_writer.write(check_msg);

    //Wait for replies & print them in DDS thread
    std::this_thread::sleep_for(std::chrono::seconds(5));

    #pragma GCC diagnostic pop

    return 0;
}
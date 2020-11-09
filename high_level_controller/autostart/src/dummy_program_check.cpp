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
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <functional>

#include <dds/pub/ddspub.hpp>

#include "RemoteProgramCheck.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/TimerFD.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include "cpm/Writer.hpp"

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

    //These data structures will later be used in the LCC to regularly check if the remotely deployed program is still online
    std::mutex map_mutex;
    std::map<std::string, bool> script_running;
    std::map<std::string, bool> middleware_running;

    //Create reader and writer to check HLC answers
    dds::topic::Topic<RemoteProgramCheck> program_check_topic = cpm::get_topic<RemoteProgramCheck>(cpm::ParticipantSingleton::Instance(), "remote_program_check");
    cpm::Writer<RemoteProgramCheck> program_check_writer("remote_program_check", true, true);

    cpm::AsyncReader<RemoteProgramCheck> program_check_reader(
        [&](dds::sub::LoanedSamples<RemoteProgramCheck>& samples){
            std::lock_guard<std::mutex> lock(map_mutex);
            for(auto sample : samples)
            {
                if(sample.info().valid())
                {
                    //Only print answers
                    if ((sample.data().is_answer()))
                    {
                        //TODO: Use count! -> Only update if newest answer
                        script_running[sample.data().source_id()] = sample.data().script_running();
                        middleware_running[sample.data().source_id()] = sample.data().middleware_running();
                    }
                }
            }
        },
        cpm::ParticipantSingleton::Instance(),
        program_check_topic,
        true,
        false
    );

    //Wait for DDS to connect properly, or the first msg is lost
    std::this_thread::sleep_for(std::chrono::seconds(2));

    //Send program check request
    RemoteProgramCheck check_msg;
    check_msg.source_id("dummy_check");
    check_msg.is_answer(false);

    //Again dummy code for later implementation in the LCC
    std::vector<uint8_t> running_hlc_ids = { 1, 3, 8 }; //Will later be dynamic, depending on which HLCs the software was deployed
    //The vector should maybe also be updated if a NUC went offline/crashed, but reporting this twice would also not be too bad 
    //(Report would then be: Programs XY have crashed, NUC crashed -> probably even better this way)


    for (uint8_t i = 0; i < 255; ++i) //LCC: Make this an infinite loop within a thread that can be stopped from outside (with an atomic variable)
    {
        check_msg.count(i);
        program_check_writer.write(check_msg);
        std::cout << "Msg written" << std::endl;

        //Wait for replies before sending the next msg
        std::this_thread::sleep_for(std::chrono::seconds(2));

        //Check for answers for all IDs, in LCC: make sure that some time has passed before missing answers are being reported 
        //(from testing experience: in the beginning, the connection is not stable enough to assume that the other party is offline,
        //and there are tests for offline NUCs already)
        for (auto id_iterator = running_hlc_ids.begin(); id_iterator != running_hlc_ids.end();)
        {
            auto id_string = std::to_string(static_cast<int>(*id_iterator));

            std::lock_guard<std::mutex> lock(map_mutex);
            if (script_running.find(id_string) != script_running.end())
            {
                //Entry exists in both maps, check if the script and the middleware are still running
                if (middleware_running.at(id_string) && script_running.at(id_string))
                {
                    ++id_iterator;
                }
                else
                {
                    if (! script_running.at(id_string))
                    {
                        cpm::Logging::Instance().write("Script crashed on NUC %s (remote)", id_string.c_str());
                    }

                    if (! middleware_running.at(id_string))
                    {
                        cpm::Logging::Instance().write("Middleware crashed on NUC %s (remote)", id_string.c_str());
                    }

                    //Erase entry s.t. the message does not appear again
                    id_iterator = running_hlc_ids.erase(id_iterator);
                }

                //Delete entry s.t. we do not assume next time that we received an answer
                script_running.erase(id_string);
                middleware_running.erase(id_string);
            }
            else
            {
                cpm::Logging::Instance().write("NUC %s did not respond to program online check", id_string.c_str());
                //Maybe actually remove IDs of offline NUCs to prevent confusion regarding this msg
            }
        }
    }

    #pragma GCC diagnostic pop

    return 0;
}
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
 * \brief This file includes a mechanism for a NUC to tell the LCC that it is online (should be called on NUC startup by a startup script, see lab_autostart.bash)
 */

#include <memory>
#include <sstream>
#include <string>
#include <functional>

#include <dds/pub/ddspub.hpp>

#include "ReadyStatus.hpp"

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

int main (int argc, char *argv[]) { 
    //Initialize the cpm logger, set domain id etc
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("hlc_hello"); 

    uint64_t callback_period = 1000000000ull;

    //Initialize the timer
    std::shared_ptr<cpm::Timer> timer = std::make_shared<cpm::TimerFD>("hlc_timer", callback_period, 0, false);

    //Create DataWriter that sends ready messages to the Lab
    dds::pub::DataWriter<ReadyStatus> writer_readyMessage
    (
        dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
        cpm::get_topic<ReadyStatus>("hlc_startup")
    );

    //Wait a bit (10 seconds) for the NUC to get its IP address; the NUCs ID can be read from its IP
    //usleep(10000000);

    //Get all IP addresses; repeat until there is a valid address, log error whenever none is found
    //Example: ID 13 -> 192.168.1.213
    std::string mask = "192.168.1.2";
    std::string hlc_id = "";
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *address;

    while (hlc_id == "")
    {
        getifaddrs(&ifap);
        for (ifa = ifap; ifa; ifa = ifa->ifa_next)
        {
            if(ifa->ifa_addr && ifa->ifa_addr->sa_family==AF_INET)
            {
                sa = (struct sockaddr_in *) ifa->ifa_addr;
                address = inet_ntoa(sa->sin_addr);

                //Get address string
                if(address != NULL)
                {
                    std::string addr(address);
                    auto pos = addr.find(mask);
                    if (pos != std::string::npos) {
                        hlc_id = addr.substr(pos + mask.size(), hlc_id.size() - pos - mask.size());
                        break;
                    }
                }
            }
        }
        freeifaddrs(ifap);

        //Log error if the own ID could not yet be determined
        if (hlc_id == "")
        {
            cpm::Logging::Instance().write(
                1,
                "%s", 
                "ID of a NUC could not yet be determined"
            );
        }
    }

    std::cout << "Set ID to " << hlc_id << std::endl;


    //Create ready message
    ReadyStatus ready_message;
    ready_message.source_id(hlc_id);

    //Suppress warning for unused parameter in timer (because we only want to show relevant warnings)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"

    timer->start([&](uint64_t t_now) {
        writer_readyMessage.write(ready_message);
    },
    [](){
        //Ignore stop signals
    });

    #pragma GCC diagnostic pop

    return 0;
}
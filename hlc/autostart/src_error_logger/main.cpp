/**
 * \class main.cpp
 * \brief This file includes a mechanism for a NUC to tell the LCC that an error occured (not all files are there that should have been published by the master PC)
 * -> This software is compiled "self-contained", it includes the cpm library (static linking) rather than linking to it dynamically
 * -> This software already should be installed on the NUCs, in the same folder as lab_autostart.bash 
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

    //Write a message every 10 seconds telling that this NUC does not have all required packages
    uint64_t callback_period = 10000000000ull;

    //Initialize the timer
    std::shared_ptr<cpm::Timer> timer = std::make_shared<cpm::TimerFD>("hlc_timer", callback_period, 0, false);

    //Same as for the autostart - program that tells the LCC that the NUC is online
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
            cpm::Logging::Instance().write("ID of a NUC could not yet be determined");
        }
    }

    std::cout << "Set ID to " << hlc_id << std::endl;

    timer->start([&](uint64_t t_now) {
        
    },
    [](){
        cpm::Logging::Instance().write("NUC %s could not download all required packages from the master PC (maybe some are missing?)", hlc_id);
    });

    return 0;
}
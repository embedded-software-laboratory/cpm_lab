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

    std::cout << "Set ID to " << hlc_id << std::endl;


    //Create ready message
    ReadyStatus ready_message;
    ready_message.source_id(hlc_id);

    timer->start([&](uint64_t t_now) {
        writer_readyMessage.write(ready_message);
    },
    [](){
        //Ignore stop signals
    });

    return 0;
}
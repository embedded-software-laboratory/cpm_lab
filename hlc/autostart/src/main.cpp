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

    //Create ready message
    ReadyStatus ready_message;
    ready_message.source_id("");

    timer->start_async([&](uint64_t t_now) {
        writer_readyMessage.write(ready_message);
    },
    [](){
        //Ignore stop signals
    });
}
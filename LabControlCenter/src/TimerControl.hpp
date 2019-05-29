#pragma once

#include <map>

#include "dds/pub/ddspub.hpp"
#include "dds/core/ddscore.hpp"

#include "cpm/AsyncReader.hpp"

#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

class TimerControl {
public: 
    TimerControl(bool use_simulated_time);

    //Start / stop functions for the global timer control
    void start();
    void stop();

private:
    //Current timestep in case simulated time is used
    uint64_t current_simulated_time;

    //Reader / writer for communication of current timestep (simulated time) and to receive start / send stop signal
    cpm::AsyncReader<ReadyStatus> ready_status_reader;
    dds::topic::Topic<SystemTrigger> trigger_topic;
    dds::pub::DataWriter<SystemTrigger> trigger_writer;

    //Data structures to save last responses and last response time

};
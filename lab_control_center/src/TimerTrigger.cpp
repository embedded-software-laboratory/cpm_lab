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

#include "TimerTrigger.hpp"

using namespace std::placeholders;
TimerTrigger::TimerTrigger(bool simulated_time) :
    use_simulated_time(simulated_time),
    /*Set up communication*/
    ready_status_reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), cpm::get_topic<ReadyStatus>("readyStatus"), dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll()),
    system_trigger_writer("systemTrigger", true)
{    
    current_simulated_time = 0;
    simulation_started.store(false);

    timer_running.store(true);

    //Create timer thread that handles receiving + sending timing messages in a more ordered fashion
    next_signal_thread = std::thread([&] () {
        //Get initial messages so that the UI displays all participants that have sent an initial ready message
        //This is also used in a real time scenario
        while(!simulation_started.load() && timer_running.load()) {
            obtain_new_ready_signals();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        while(timer_running.load()) {
            //Check if any of the participants that were waiting for a signal of the current timestep have sent an answer - if new messages were received, wait for more messages that might arrive within x milliseconds for a more ordered event handling
            while(obtain_new_ready_signals()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            //Progress to the next timestep or send the current timestep again if some participants have not sent anything yet (simulated time only)
            check_signals_and_send_next_signal();   

            //Now only continue if new messages are received
            //If waiting time gets too long, investigate
            unsigned int count = 0;
            while(!obtain_new_ready_signals()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                ++count;

                //Each second of waiting, log which participants we are still waiting for
                if (count > 100)
                {
                    count = 0;

                    //Log all participants that are still working or out of sync
                    std::stringstream id_stream;
                    std::lock_guard<std::mutex> lock(ready_status_storage_mutex);
                    std::lock_guard<std::mutex> lock2(simulated_time_mutex);
                    for (auto& entry : ready_status_storage)
                    {
                        if (entry.second.next_timestep <= current_simulated_time)
                        {
                            id_stream << " | " << entry.first;
                        }
                    }

                    cpm::Logging::Instance().write(2, "Simulated time - Participants that need to answer or are out of sync: %s", id_stream.str().c_str());
                }
            }   
        }  
    });
}

TimerTrigger::~TimerTrigger() {
    timer_running.store(false);
    if(next_signal_thread.joinable()) {
        next_signal_thread.join();
    }
}

bool TimerTrigger::obtain_new_ready_signals() {
    bool any_message_received = false; 
    
    for (auto sample : ready_status_reader.take()) {
        if (sample.info().valid()) {
            any_message_received = true;
            
            //Data from the sample to string
            const std::string id = sample.data().source_id();

            std::unique_lock<std::mutex> lock(ready_status_storage_mutex);
            std::unique_lock<std::mutex> lock2(simulated_time_mutex);

            //Save current start request and storage start request of the participant (the latter may be higher)
            uint64_t next_start_request = sample.data().next_start_stamp().nanoseconds();
            uint64_t storage_start_request = 0;
            if (ready_status_storage.find(id) != ready_status_storage.end()) {
                storage_start_request = ready_status_storage[id].next_timestep;
            }

            //Only store new data if the current timestep is higher than the timestep that was stored for the participant
            if (next_start_request >= storage_start_request) {
                //The LCC is only waiting for a response if:
                //a) TODO The participant has been pre-registered and has not yet sent any message -> Kommentar entfernen?
                //b) Simulated time is used - then the timer needs to wait for all participants that have registered for the timestep
                ParticipantStatus current_participant_status;
                //The LCC is waiting for a response if the participant registered a "callback" in the current timestep
                if (next_start_request == current_simulated_time && use_simulated_time) {
                    current_participant_status = WORKING;
                } //If an old message is received and the entry in the storage is old as well, the participant is out of sync
                else if (next_start_request < current_simulated_time && use_simulated_time) {
                    current_participant_status = OUT_OF_SYNC;
                    cpm::Logging::Instance().write(1, "Participant with id '%s' is out of sync", id.c_str());
                }
                else if (next_start_request > current_simulated_time && use_simulated_time) {
                    current_participant_status = WAITING;
                }
                else {
                    current_participant_status = REALTIME;
                }

                TimerData data;
                data.last_message_receive_stamp = cpm::get_time_ns();
                data.next_timestep = next_start_request;
                data.participant_status = current_participant_status;

                ready_status_storage[id] = data;
            }
            else {
                cpm::Logging::Instance().write(
                    1,
                    "LCC Timer: Received old timestamp from participant with ID %s", 
                    id.c_str()
                );
            }
        }
    }

    return any_message_received;

    //TODO Check if uint64_t max number is close and stop the program automatically
    //samples.return_loan(); Actually made a difference in one situation
}

void TimerTrigger::send_start_signal() {
    if (use_simulated_time) {
        simulation_started.store(true);
    }
    else {
        SystemTrigger trigger;

        trigger.next_start(TimeStamp(cpm::get_time_ns() + 1000000000ull));
        system_trigger_writer.write(trigger);

        //New messages are now meaningless, thus shut down the message receiver
        timer_running.store(false);
        //TODO What if the button is pressed before any participant sent a message?
    }
}

bool TimerTrigger::check_signals_and_send_next_signal() {
    if (use_simulated_time && timer_running.load()) {
        //Find smallest next time step in the storage
        uint64_t next_simulated_time = 0;
        bool has_data = false;
        std::unique_lock<std::mutex> storage_lock(ready_status_storage_mutex);
        for (auto const& pair : ready_status_storage) {
            if (!has_data || next_simulated_time > pair.second.next_timestep) {
                next_simulated_time = pair.second.next_timestep;
                has_data = true;
            }
        }
        storage_lock.unlock();

        std::lock_guard<std::mutex> lock(simulated_time_mutex);
        //React according to current data
        if (!has_data) {
            cpm::Logging::Instance().write(
                1,
                "%s", 
                "LCC Timer: No data or only invalid data received!"
            );
        }
        else if (next_simulated_time < current_simulated_time) {
            cpm::Logging::Instance().write(
                1,
                "%s", 
                "LCC Timer: At least one participant is out of sync (or its answer was not received)!"
            );
        }
        else {
            //The timer can progress to the next smallest timestep as all participants have performed their computations
            //Or: The current timestep is kept as they are equal and feedback from some participants is still required
            current_simulated_time = next_simulated_time;

            //Set all participants to "working" that waited for this message
            std::lock_guard<std::mutex> participant_lock(ready_status_storage_mutex);
            for (auto& pair : ready_status_storage) {
                if (pair.second.next_timestep == current_simulated_time) {
                    pair.second.participant_status = ParticipantStatus::WORKING;
                }
            }

            //Send system trigger message to participants
            SystemTrigger trigger;
            trigger.next_start(TimeStamp(current_simulated_time));
            system_trigger_writer.write(trigger);

            return true;
        }
    }

    return false;
}

void TimerTrigger::send_stop_signal() {
    SystemTrigger trigger;
    trigger.next_start(TimeStamp(cpm::TRIGGER_STOP_SYMBOL));
    system_trigger_writer.write(trigger);

    cpm::Logging::Instance().write(
        1,
        "%s", 
        "LCC: Sent stop signal"
    );

    timer_running.store(false);
    if(next_signal_thread.joinable()) {
        next_signal_thread.join();
    }
}

std::map<string, TimerData> TimerTrigger::get_participant_message_data() {
    std::lock_guard<std::mutex> lock(ready_status_storage_mutex);
    std::map<string, TimerData> map_copy = ready_status_storage;
    return map_copy;
}

void TimerTrigger::get_current_simulated_time(bool& _use_simulated_time, uint64_t& _current_time) {
    bool simulated_copy = use_simulated_time;
    _use_simulated_time = simulated_copy;

    std::lock_guard<std::mutex> lock(simulated_time_mutex);
    uint64_t time_copy = current_simulated_time;
    _current_time = time_copy;
}
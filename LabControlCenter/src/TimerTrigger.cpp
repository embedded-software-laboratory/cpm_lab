#include "TimerTrigger.hpp"

using namespace std::placeholders;
TimerTrigger::TimerTrigger(bool simulated_time) :
    use_simulated_time(simulated_time),
    /*Set up communication*/
    ready_status_reader(std::bind(&TimerTrigger::ready_status_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<ReadyStatus>("ready"), true),
    system_trigger_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<SystemTrigger>("system_trigger"), dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable())
{    
    current_simulated_time = 0;
    timer_running.store(false);
}

void TimerTrigger::ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples) {
    bool any_message_received = false; 
    
    for (auto sample : samples) {
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
                cpm::Logging::Instance().write("LCC Timer: Received old timestamp from participant with ID %s", id.c_str());
            }
        }
    }

    //Check if any of the participants that were waiting for a signal of the current timestep have sent an answer - in that case, progress to the next timestep or send the current timestep again if some participants have not sent anything yet (simulated time only)
    if (any_message_received) {
        check_signals_and_send_next_signal();
    }

    //TODO Check if uint64_t max number is close and stop the program automatically
    //samples.return_loan(); Actually made a difference in one situation
}

void TimerTrigger::send_start_signal() {
    timer_running.store(true);

    if (use_simulated_time) {
        bool signal_sent = check_signals_and_send_next_signal();
        //TODO Do something if no signal was sent
    }
    else {
        SystemTrigger trigger;

        trigger.next_start(TimeStamp(cpm::get_time_ns() + 1000000000ull));
        system_trigger_writer.write(trigger);
        //TODO What if the button is pressed before any participant sent a message?
    }
}

bool TimerTrigger::check_signals_and_send_next_signal() {
    if (use_simulated_time && timer_running.load()) {
        //Find smallest next time step in the storage
        uint64_t next_simulated_time = 0;
        bool has_data = false;
        for (auto const& pair : ready_status_storage) {
            if (!has_data || next_simulated_time > pair.second.next_timestep) {
                next_simulated_time = pair.second.next_timestep;
                has_data = true;
            }
        }

        std::lock_guard<std::mutex> lock(simulated_time_mutex);
        //React according to current data
        if (!has_data) {
            cpm::Logging::Instance().write("LCC Timer: No data or only invalid data received!");
        }
        else if (next_simulated_time < current_simulated_time) {
            cpm::Logging::Instance().write("LCC Timer: At least one participant is out of sync (or its answer was not received)!");
        }
        else {
            //The timer can progress to the next smallest timestep as all participants have performed their computations
            //Or: The current timestep is kept as they are equal and feedback from some participants is still required
            current_simulated_time = next_simulated_time;

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
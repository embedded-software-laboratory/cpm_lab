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
    bool all_invalid = true; 
    
    for (auto sample : samples) {
        if (sample.info().valid()) {
            all_invalid = false;
            
            //Data from the sample to string
            std::string id = sample.data().source_id();
            //Find out the last message timestamp and print it to the UI - this can be useful for debugging purposes
            std::string last_message = get_current_realtime();

            std::unique_lock<std::mutex> lock(ready_status_storage_mutex);
            std::unique_lock<std::mutex> lock2(simulated_time_mutex);

            //The LCC is only waiting for a response if:
            //a) TODO The participant has been pre-registered and has not yet sent any message
            //b) Simulated time is used - then the timer needs to wait for all participants that have registered for the timestep
            std::string waiting_response;
            if (sample.data().next_start_stamp().nanoseconds() == current_simulated_time && use_simulated_time) {
                if (ready_status_storage.find(id) == ready_status_storage.end() || ready_status_storage[id].next_timestep == current_simulated_time){
                    waiting_response = "YES";
                }
            }
            else if (sample.data().next_start_stamp().nanoseconds() < current_simulated_time && use_simulated_time) {
                if (ready_status_storage.find(id) == ready_status_storage.end() || ready_status_storage[id].next_timestep < current_simulated_time){
                    waiting_response = "OUT OF SYNC";
                }
            }
            else {
                waiting_response = "-";
            }

            //Only store new data if the current timestep is higher than the timestep that was stored for the vehicle
            uint64_t next_step;
            if (ready_status_storage.find(id) == ready_status_storage.end()) {
                next_step = sample.data().next_start_stamp().nanoseconds();
            }
            else if (ready_status_storage[id].next_timestep <= sample.data().next_start_stamp().nanoseconds()) {
                next_step = sample.data().next_start_stamp().nanoseconds();
            }
            else {
                cpm::Logging::Instance().write("LCC Timer: Received old timestamp from participant with ID %s", id.c_str());
            }

            TimerData data;
            data.last_message = last_message;
            data.waiting_for_response = waiting_response;
            data.next_timestep = next_step;

            ready_status_storage[id] = data;

            lock.unlock();
            lock2.unlock();
        }
    }

    //Check if all vehicles that were waiting for a signal of the current timestep have sent an answer - in that case, progress to the next timestep (simulated time only)
    if (!all_invalid) {
        send_next_signal();
    }

    //TODO Check if uint64_t max number is close and stop the program automatically
}

void TimerTrigger::send_start_signal() {
    timer_running.store(true);

    if (use_simulated_time) {
        bool signal_sent = send_next_signal();
        //TODO Do something if no signal was sent
    }
    else {
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(0));
        system_trigger_writer.write(trigger);
        //TODO What if the button is pressed before any participant sent a message?
    }
}

bool TimerTrigger::send_next_signal() {
    if (use_simulated_time && timer_running.load()) {
        //Find smallest next time step in the storage
        uint64_t smallest_step = 0;
        bool has_data = false;
        for (auto const& pair : ready_status_storage) {
            if (smallest_step == 0 || smallest_step > pair.second.next_timestep) {
                smallest_step = pair.second.next_timestep;
                has_data = true;
            }
        }

        std::lock_guard<std::mutex> lock(simulated_time_mutex);
        //React according to current data
        if (!has_data) {
            cpm::Logging::Instance().write("LCC Timer: No data or only invalid data received!");
        }
        else if (smallest_step < current_simulated_time) {
            cpm::Logging::Instance().write("LCC Timer: At least one participant is out of sync (or its answer was not received)!");
        }
        else if (smallest_step == current_simulated_time) {
            cpm::Logging::Instance().write("LCC Timer: Some participants are still in the current timestep, waiting for an answer...");
        }
        else {
            //The timer can progress to the next smallest timestep as all participants have performed their computations
            current_simulated_time = smallest_step;

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

std::string TimerTrigger::get_current_realtime() {
    std::stringstream time_stream;
    std::time_t t = std::time(0);
    std::tm* time = std::localtime(&t);

    time_stream << time->tm_hour << ":" << time->tm_min << ":" << time->tm_sec;
    return time_stream.str();
}

std::map<string, TimerData> TimerTrigger::get_participant_message_data() {
    std::lock_guard<std::mutex> lock(ready_status_storage_mutex);
    std::map<string, TimerData> map_copy = ready_status_storage;
    return map_copy;
}

void TimerTrigger::get_current_simulated_time(bool& _use_simulated_time, uint64_t& _current_time) {
    uint64_t simulated_copy = use_simulated_time;
    _use_simulated_time = simulated_copy;

    std::lock_guard<std::mutex> lock(simulated_time_mutex);
    uint64_t time_copy = current_simulated_time;
    _current_time = time_copy;
}
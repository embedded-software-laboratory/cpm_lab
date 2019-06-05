#include "TimerViewUI.hpp"

#define TRIGGER_STOP_SYMBOL (0xffffffffffffffffull)

using namespace std::placeholders;
TimerViewUI::TimerViewUI(bool simulated_time) :
    use_simulated_time(simulated_time),
    /*Set up communication*/
    ready_status_reader(std::bind(&TimerViewUI::ready_status_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<ReadyStatus>("ready"), true),
    system_trigger_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<SystemTrigger>("system_trigger"), dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable())
 {
    ui_builder = Gtk::Builder::create_from_file("ui/timer/timer.glade");

    ui_builder->get_widget("parent", parent);
    ui_builder->get_widget("button_start", button_start);
    ui_builder->get_widget("button_stop", button_stop);
    ui_builder->get_widget("active_timers_treeview", active_timers_treeview);
    ui_builder->get_widget("current_timestep_label", current_timestep_label);

    assert(parent);
    assert(button_start);
    assert(button_stop);
    assert(active_timers_treeview);
    assert(current_timestep_label);

    //Create model for view
    timer_list_storage = Gtk::ListStore::create(timer_record);
    active_timers_treeview->set_model(timer_list_storage);

    //Use model_record, add it to the view
    active_timers_treeview->append_column("ID", timer_record.column_id);
    active_timers_treeview->append_column("Last message", timer_record.column_last_message);
    active_timers_treeview->append_column("Waiting for response", timer_record.column_waiting_for_response);
    active_timers_treeview->append_column("Next timestep", timer_record.column_next_step);

    //Set equal width for all columns
    for (int i = 0; i < 4; ++i) {
        active_timers_treeview->get_column(i)->set_resizable(true);
        active_timers_treeview->get_column(i)->set_min_width(20);
        active_timers_treeview->get_column(i)->set_fixed_width(50);
        active_timers_treeview->get_column(i)->set_expand(true);
    }

    //Register callbacks for button presses
    button_start->signal_clicked().connect(sigc::mem_fun(this, &TimerViewUI::send_start_signal));
    button_stop->signal_clicked().connect(sigc::mem_fun(this, &TimerViewUI::send_stop_signal));
}

void TimerViewUI::insert_or_change_treeview(std::string id_string, std::string last_message_string, std::string waiting_response_string, std::string next_step_string) {
        Glib::ustring id_ustring(id_string);
        Glib::ustring last_message_ustring(last_message_string);
        Glib::ustring waiting_response_ustring(waiting_response_string);
        Glib::ustring next_step_ustring(next_step_string);

        Gtk::TreeModel::Row row;
        bool entry_exists = false;

        //Search if the entry already exists
        for (Gtk::TreeModel::iterator iter = timer_list_storage->children().begin(); iter != timer_list_storage->children().end(); ++iter) {
            row = *iter;
            if (row[timer_record.column_id] == id_ustring) {
                entry_exists = true;
                break;
            }
        }

        //Else create a new entry
        if (!entry_exists) {
            row = *(timer_list_storage->append());
        }

        row[timer_record.column_id] = id_ustring;
        row[timer_record.column_last_message] = last_message_ustring;
        row[timer_record.column_waiting_for_response] = waiting_response_ustring;
        row[timer_record.column_next_step] = next_step_ustring;
}

void TimerViewUI::ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            //Data from the sample to string
            std::string id = sample.data().source_id();
            std::stringstream step_stream;
            step_stream << sample.data().next_start_stamp().nanoseconds();
            std::string next_step = step_stream.str();
            //Find out the last message timestamp and print it to the UI - this can be useful for debugging purposes
            std::string last_message = get_current_realtime();

            std::unique_lock<std::mutex> lock(ready_status_storage_mutex);

            //The LCC is only waiting for a response if:
            //a) TODO The participant has been pre-registered and has not yet sent any message
            //b) Simulated time is used - then the timer needs to wait for all participants that have registered for the timestep
            std::string waiting_response;
            if (sample.data().next_start_stamp().nanoseconds() == current_simulated_time && use_simulated_time) {
                if (ready_status_storage.find(id) == ready_status_storage.end() || ready_status_storage[id] == current_simulated_time){
                    waiting_response = "YES";
                }
            }
            else if (sample.data().next_start_stamp().nanoseconds() < current_simulated_time && use_simulated_time) {
                if (ready_status_storage.find(id) == ready_status_storage.end() || ready_status_storage[id] < current_simulated_time){
                    waiting_response = "OUT OF SYNC";
                }
            }
            else {
                waiting_response = "-";
            }

            //Only store new data if the current timestep is higher than the timestep that was stored for the vehicle
            if (ready_status_storage.find(id) == ready_status_storage.end()) {
                ready_status_storage[id] = sample.data().next_start_stamp().nanoseconds();
            }
            else if (ready_status_storage[id] < sample.data().next_start_stamp().nanoseconds()) {
                ready_status_storage[id] = sample.data().next_start_stamp().nanoseconds();
            }
            else {
                std::stringstream step_stream;
                step_stream << ready_status_storage[id] << "()";
                next_step = step_stream.str();

                cpm::Logging::Instance().write("LCC Timer: Received old timestamp from participant with ID %s", id);
            }

            lock.unlock();

            //Also update the UI (in case last_message, waiting_response or next_step have changed or if a new participant needs to be added)
            //TODO: Update the UI in discrete steps for performance reasons?
            insert_or_change_treeview(id, last_message, waiting_response, next_step);
        }
    }

    //Check if all vehicles that were waiting for a signal of the current timestep have sent an answer - in that case, progress to the next timestep (simulated time only)
    send_next_signal();

    //TODO Check if uint64_t max number is close and stop the program automatically
}

void TimerViewUI::send_start_signal() {
    timer_running = true;

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

    button_start->set_sensitive(false);
}

bool TimerViewUI::send_next_signal() {
    if (use_simulated_time && timer_running) {
        //Find smallest next time step in the storage
        uint64_t smallest_step = 0;
        bool has_data = false;
        for (auto const& pair : ready_status_storage) {
            if (smallest_step == 0 || smallest_step > pair.second) {
                smallest_step = pair.second;
                has_data = true;
            }
        }

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
            //Update current time in UI
            std::stringstream time_stream;
            time_stream << current_simulated_time;
            Glib::ustring step_ustring(time_stream.str());
            current_timestep_label->set_label(step_ustring);

            //Send system trigger message to participants
            SystemTrigger trigger;
            trigger.next_start(TimeStamp(current_simulated_time));
            system_trigger_writer.write(trigger);

            return true;
        }
    }

    return false;
}

void TimerViewUI::send_stop_signal() {
        SystemTrigger trigger;
        trigger.next_start(TimeStamp(TRIGGER_STOP_SYMBOL)); //TODO: Symbol should be part of cpm lib
        system_trigger_writer.write(trigger);
}

std::string TimerViewUI::get_current_realtime() {
    std::stringstream time_stream;
    std::time_t t = std::time(0);
    std::tm* time = std::localtime(&t);

    time_stream << time->tm_hour << ":" << time->tm_min << ":" << time->tm_sec;
    return time_stream.str();
}

Gtk::Widget* TimerViewUI::get_parent() {
    return parent;
}
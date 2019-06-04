#include "TimerViewUI.hpp"

using namespace std::placeholders;
TimerViewUI::TimerViewUI(bool simulated_time) :
    use_simulated_time(simulated_time),
    /*Set up communication*/
    ready_status_reader(std::bind(&TimerViewUI::ready_status_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic("ready"), true),
    system_trigger_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic("system_trigger"))
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
}

void TimerViewUI::ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {

        }
    }
}

Gtk::Widget* TimerViewUI::get_parent() {
    return parent;
}
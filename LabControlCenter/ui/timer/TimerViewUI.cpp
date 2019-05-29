#include "TimerViewUI.hpp"

TimerViewUI::TimerViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI) :
    vehicle_manual_control_ui(vehicleManualControlUi),
    param_view_ui(paramViewUI)
 {
    ui_builder = Gtk::Builder::create_from_file("ui/timer/timer.glade");

    ui_builder->get_widget("parent", parent);
    ui_builder->get_widget("button_start", button_start);
    ui_builder->get_widget("button_stop", button_stop);
    ui_builder->get_widget("active_timers_treeview", active_timers_treeview);
    ui_builder->get_widget("received_messages_treeview", received_messages_treeview);
    ui_builder->get_widget("current_timestep_label", current_timestep_label);

    assert(parent);
    assert(button_start);
    assert(button_stop);
    assert(active_timers_treeview);
    assert(received_messages_treeview);
    assert(current_timestep_label);
}

Gtk::Widget* TimerViewUI::get_parent() {
    return parent;
}
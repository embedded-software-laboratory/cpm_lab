#pragma once

#include <memory>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"

class TabsViewUI {
private:
    Gtk::Notebook* right_notebook;
    Gtk::Label* manual_control_label;
    Gtk::Label* parameters_label;

public:
    TabsViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI);
    Gtk::Notebook* get_parent();
}
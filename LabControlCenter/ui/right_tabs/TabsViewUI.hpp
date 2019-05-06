#pragma once

#include "defaults.hpp"
#include <cassert>
#include <memory>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"

class TabsViewUI {
private:
    Glib::RefPtr<Gtk::Builder> tabs_builder;
    std::shared_ptr<VehicleManualControlUi> vehicle_manual_control_ui;
    std::shared_ptr<ParamViewUI> param_view_ui;
    Gtk::Notebook* right_notebook;

public:
    TabsViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI);
    std::shared_ptr<ParamViewUI> get_param_view();
    bool manual_control_page_active();
    bool param_page_active();
    Gtk::Widget* get_parent();
};
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

    Gtk::Notebook* right_notebook;

public:
    TabsViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI);
    Gtk::Widget* get_parent();
};
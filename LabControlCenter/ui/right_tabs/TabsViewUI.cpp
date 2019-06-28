#include "TabsViewUI.hpp"

TabsViewUI::TabsViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI) :
    vehicle_manual_control_ui(vehicleManualControlUi),
    param_view_ui(paramViewUI)
 {
    tabs_builder = Gtk::Builder::create_from_file("ui/right_tabs/right_tabs.glade");

    tabs_builder->get_widget("right_notebook", right_notebook);

    assert(right_notebook);

    Glib::ustring manual_control_label("Manual Control");
    Glib::ustring parameters_label("Parameters");

    right_notebook->insert_page(*(vehicleManualControlUi->get_parent()), manual_control_label, -1);
    right_notebook->insert_page(*(paramViewUI->get_parent()), parameters_label, -1);
}

std::shared_ptr<ParamViewUI> TabsViewUI::get_param_view() {
    return param_view_ui;
}

bool TabsViewUI::manual_control_page_active() {
    return right_notebook->get_current_page() == 0;
}

bool TabsViewUI::param_page_active() {
    return right_notebook->get_current_page() == 1;
}

Gtk::Widget* TabsViewUI::get_parent() {
    return right_notebook;
}
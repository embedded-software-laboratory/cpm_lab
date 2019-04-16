#include "TabsViewUI.hpp"

TabsViewUI::TabsViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI) {
    Glib::RefPtr<Gtk::Builder> tabs_builder = Gtk::Builder::create_from_file("ui/right_tabs/right_tabs.glade");

    tabs_builder->get_widget("right_notebook", right_notebook);
    tabs_builder->get_widget("parameters_label", parameters_label);
    tabs_builder->get_widget("manual_control_label", manual_control_label);

    assert(right_notebook);
    assert(parameters_label);
    assert(manual_control_label);

    gtk_notebook_insert_page(right_notebook, vehicleManualControlUi->get_parent(), manual_control_label, 0);
    gtk_notebook_insert_page(right_notebook, paramViewUI->get_parent() parameters_label, 1);
}

Gtk::Notebook* TabsViewUI::get_parent() {
    return right_notebook;
}
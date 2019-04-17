#include "TabsViewUI.hpp"

TabsViewUI::TabsViewUI(std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, std::shared_ptr<ParamViewUI> paramViewUI) {
    tabs_builder = Gtk::Builder::create_from_file("ui/right_tabs/right_tabs.glade");

    tabs_builder->get_widget("right_notebook", right_notebook);

    assert(right_notebook);

    std::cout << "Building notebook..." << std::endl;

    Glib::ustring manual_control_label("Manual Control");
    Glib::ustring parameters_label("Parameters");

    std::cout << "Made strings..." << std::endl;

    right_notebook->insert_page(*(vehicleManualControlUi->get_parent()), manual_control_label, -1);

    std::cout << "Inserted manual control page..." << std::endl;

    right_notebook->insert_page(*(paramViewUI->get_parent()), parameters_label, -1);

    std::cout << "Notebook built" << std::endl;
}

Gtk::Widget* TabsViewUI::get_parent() {
    return right_notebook;
}
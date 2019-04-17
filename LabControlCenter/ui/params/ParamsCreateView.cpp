#include "ParamsCreateView.hpp"

ParamsCreateView::ParamsCreateView() {
    params_create_builder = Gtk::Builder::create_from_file("ui/params/params_create.glade");

    params_create_builder->get_widget("params_create_dialog", parent);
}

Gtk::Widget* ParamsCreateView::get_parent() {
    return parent;
}
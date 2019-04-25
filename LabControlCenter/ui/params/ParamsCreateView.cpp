#include "ParamsCreateView.hpp"

ParamsCreateView::ParamsCreateView(std::function<void(ParameterWithDescription, bool)> _on_close_callback) :
    on_close_callback(_on_close_callback)
{
    init_members();

    //Create empty fields - TODO field types depend on input type, listener for type change, also TODO: unify with other constructor when final structure is known
    create_inputs();
}

ParamsCreateView::ParamsCreateView(std::function<void(ParameterWithDescription, bool)> _on_close_callback, ParameterWithDescription param) :
    on_close_callback(_on_close_callback)
{
    init_members();

    params_create_add_button->set_label("Save");

    //Create fields with given values - TODO field types depend on input type, listener for type change
    create_inputs();
}

void ParamsCreateView::create_inputs() {
    std::string name;
    std::string type;
    std::string value;
    std::string info;

    ParameterWithDescription::parameter_to_string(param, name, type, value, info);

    Glib::ustring name_ustring = name;
    Glib::ustring type_ustring = type;
    Glib::ustring value_ustring = value;
    Glib::ustring info_ustring = info;

    name_entry = Gtk::manage(new Gtk::Entry());
    type_entry = Gtk::manage(new Gtk::Entry());
    value_entry = Gtk::manage(new Gtk::Entry());
    info_entry = Gtk::manage(new Gtk::Entry());
    
    name_entry->set_hexpand(true);
    name_entry->set_vexpand(true);
    type_entry->set_hexpand(true);
    type_entry->set_vexpand(true);
    value_entry->set_hexpand(true);
    value_entry->set_vexpand(true);
    info_entry->set_hexpand(true);
    info_entry->set_vexpand(true);

    name_entry->set_text(name_ustring);
    type_entry->set_text(type_ustring);
    value_entry->set_text(value_ustring);
    info_entry->set_text(info_ustring);

    params_create_values_grid->attach(*name_entry, 1, 0, 1, 1);
    params_create_values_grid->attach(*type_entry, 1, 1, 1, 1);
    params_create_values_grid->attach(*value_entry, 1, 2, 1, 1);
    params_create_values_grid->attach(*info_entry, 1, 3, 1, 1);
    params_create_values_grid->show_all_children();
}

void create_value_input() {

}

void ParamsCreateView::init_members() {
    params_create_builder = Gtk::Builder::create_from_file("ui/params/params_create.glade");

    params_create_builder->get_widget("params_create_dialog", parent);
    params_create_builder->get_widget("params_create_dialog", window);
    params_create_builder->get_widget("params_create_box", params_create_box);
    params_create_builder->get_widget("params_create_buttons", params_create_buttons);
    params_create_builder->get_widget("params_create_abort_button", params_create_abort_button);
    params_create_builder->get_widget("params_create_add_button", params_create_add_button);
    params_create_builder->get_widget("params_create_values_grid", params_create_values_grid);

    assert(parent);
    assert(window);
    assert(params_create_box);
    assert(params_create_buttons);
    assert(params_create_abort_button);
    assert(params_create_add_button);
    assert(params_create_values_grid);

    //Set values so that the other cannot be used until the parameter is set
    window->set_deletable(false); //No close button, user must use "abort" or "add"
    window->show();

    params_create_abort_button->signal_clicked().connect(sigc::mem_fun(this, &ParamsCreateView::on_abort));
    params_create_add_button->signal_clicked().connect(sigc::mem_fun(this, &ParamsCreateView::on_add));
}

void ParamsCreateView::on_abort() {
    window->close();
    //on_close_callback("","","",""); TODO
}

void ParamsCreateView::on_add() {
    std::string name = name_entry->get_text();
    std::string type = type_entry->get_text();
    std::string value = value_entry->get_text();
    std::string info = info_entry->get_text();

    window->close();
    //on_close_callback(name, type, value, info); TODO
}

bool ParamsCreateView::string_to_bool(std::string str) {
    if (str == "1" || str == "true") {
        return true;
    }
    else if (str == "0" || str == "false") {
        return false;
    }
    return false;
}

bool ParamsCreateView::string_to_int(std::string str) {
    return true;
}

bool ParamsCreateView::string_to_double(std::string str) {
    return true;
}

bool ParamsCreateView::string_to_int_vector(std::string str) {
    return true;
}

bool ParamsCreateView::string_to_double_vector(std::string str) {
    return true;
}
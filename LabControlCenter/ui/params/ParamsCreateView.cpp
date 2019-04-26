#include "ParamsCreateView.hpp"

ParamsCreateView::ParamsCreateView(std::function<void(ParameterWithDescription, bool)> _on_close_callback, std::function<bool(std::string)> _check_param_exists, int _float_precision) :
    on_close_callback(_on_close_callback),
    check_param_exists(_check_param_exists),
    float_precision(_float_precision),
    is_edit_window(false)
{
    init_members();

    //Create empty fields - TODO field types depend on input type, listener for type change, also TODO: unify with other constructor when final structure is known
    create_inputs();
}

ParamsCreateView::ParamsCreateView(std::function<void(ParameterWithDescription, bool)> _on_close_callback, std::function<bool(std::string)> _check_param_exists, ParameterWithDescription _param, int _float_precision) :
    on_close_callback(_on_close_callback),
    check_param_exists(_check_param_exists),
    float_precision(_float_precision),
    param(_param),
    is_edit_window(true)
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

    ParameterWithDescription::parameter_to_string(param, name, type, value, info, float_precision);

    Glib::ustring name_ustring = name;
    Glib::ustring type_ustring = type;
    Glib::ustring info_ustring = info;

    name_entry = Gtk::manage(new Gtk::Entry());
    type_entry = Gtk::manage(new Gtk::ComboBoxText());
    info_entry = Gtk::manage(new Gtk::Entry());
    
    name_entry->set_hexpand(true);
    name_entry->set_vexpand(true);
    type_entry->set_hexpand(true);
    type_entry->set_vexpand(true);
    info_entry->set_hexpand(true);
    info_entry->set_vexpand(true);

    //Type box
    type_entry->append("Bool");
    type_entry->append("Double");
    type_entry->append("Double List");
    type_entry->append("Integer");
    type_entry->append("Integer List");
    type_entry->append("String");
    if (type != "") {
        type_entry->set_active_text(type_ustring);
    }
    type_entry->signal_changed().connect(sigc::mem_fun(*this, &ParamsCreateView::on_type_changed) );

    name_entry->set_text(name_ustring);
    name_entry->signal_changed().connect(sigc::mem_fun(this, &ParamsCreateView::on_value_entry_changed));

    info_entry->set_text(info_ustring);

    params_create_values_grid->attach(*name_entry, 1, 0, 1, 1);
    params_create_values_grid->attach(*type_entry, 1, 1, 1, 1);
    params_create_values_grid->attach(*info_entry, 1, 3, 1, 1);
    params_create_values_grid->show_all_children();

    on_type_changed();
}

void ParamsCreateView::on_type_changed() {
    //To get the value string, as on_type_changed is not only called from create_inputs
    std::string name;
    std::string type;
    std::string value;
    std::string info;

    ParameterWithDescription::parameter_to_string(param, name, type, value, info, float_precision);

    Glib::ustring value_ustring = value;

    //"Delete" all input fields that currently exist
    params_create_values_grid->remove(value_entry);
    params_create_values_grid->remove(value_switch);

    //Add "optimal" input fields depending on the chosen data types
    if (type_entry->get_active_text() == "Bool") {
        value_switch.set_hexpand(true);
        value_switch.set_vexpand(true);
        value_switch.set_active(param.parameter_data.value_bool());
        params_create_values_grid->attach(value_switch, 1, 2, 1, 1);
        params_create_values_grid->show_all_children();
    }
    else {
        value_entry.set_hexpand(true);
        value_entry.set_vexpand(true);
        value_entry.set_text(value_ustring);
        value_entry.signal_changed().connect(sigc::mem_fun(this, &ParamsCreateView::on_value_entry_changed));
        params_create_values_grid->attach(value_entry, 1, 2, 1, 1);
        params_create_values_grid->show_all_children();
    }
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
    on_close_callback(param, false); //false -> do not save changes
}

void ParamsCreateView::on_add() {
    std::string name = name_entry->get_text();
    std::string type = type_entry->get_active_text();
    std::string info = info_entry->get_text();

    //If the parameter already exists and is not the parameter that should be edited, do not allow to save it
    bool param_exists = false;
    if (param.parameter_data.name() != name || !is_edit_window) {
        param_exists = check_param_exists(name);
    }

    ParameterWithDescription param;
    param.parameter_data.name(name);
    param.parameter_description = info;
    
    //If false, the param value is invalid, the window is not closed and some kind of element or color shows the user what he did wrong
    bool value_conversion_valid = false;

    if (type == "Bool") {
        param.parameter_data.type(ParameterType::Bool);
        param.parameter_data.value_bool(value_switch.get_active());
        value_conversion_valid = true;
    }
    else if (type == "Integer") {

    }
    else if (type == "Double") {
        
    }
    else if (type == "String") {
        
    }
    else if (type == "Integer List") {
        
    }
    else if (type == "Double List") {
        
    }

    //std::string value = value_entry->get_text();

    if (value_conversion_valid && !param_exists) {
        window->close();
        on_close_callback(param, true);
    }
    else {
        if (!value_conversion_valid) {
            value_entry.get_style_context()->add_class("error");
        }
        if (param_exists) {
            name_entry->get_style_context()->add_class("error");
        }
    }
}

void ParamsCreateView::on_value_entry_changed() {
    value_entry.get_style_context()->remove_class("error");
}

void ParamsCreateView::on_name_entry_changed() {
    name_entry->get_style_context()->remove_class("error");
}

bool ParamsCreateView::string_to_bool(std::string str, bool& value) {
    if (str == "1" || str == "true") {
        value = true;
        return true;
    }
    else if (str == "0" || str == "false") {
        value = false;
        return true;
    }
    return false;
}

bool ParamsCreateView::string_to_int(std::string str, int32_t& value) {
    value = 0;
    return true;
}

bool ParamsCreateView::string_to_double(std::string str, double& value) {
    value = 0.0;
    return true;
}

bool ParamsCreateView::string_to_int_vector(std::string str, std::vector<int32_t>& value) {
    value.clear();
    return true;
}

bool ParamsCreateView::string_to_double_vector(std::string str, std::vector<double>& value) {
    value.clear();
    return true;
}
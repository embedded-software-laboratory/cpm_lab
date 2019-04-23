#include "ParamsCreateView.hpp"

ParamsCreateView::ParamsCreateView(std::function<void(std::string, std::string, std::string, std::string)> _on_close_callback) :
    on_close_callback(_on_close_callback)
{
    init_members();

    //Create empty fields - TODO field types depend on input type, listener for type change, also TODO: unify with other constructor when final structure is known
    name_entry = Gtk::manage(new Gtk::Entry());
    type_entry = Gtk::manage(new Gtk::Entry());
    value_entry = Gtk::manage(new Gtk::Entry());
    info_entry = Gtk::manage(new Gtk::Entry());
    
    params_create_values_grid->attach(*name_entry, 1, 0);
    params_create_values_grid->attach(*type_entry, 1, 1);
    params_create_values_grid->attach(*value_entry, 1, 2);
    params_create_values_grid->attach(*info_entry, 1, 3);
}

ParamsCreateView::ParamsCreateView(std::function<void(std::string, std::string, std::string, std::string)> _on_close_callback, std::string _name, std::string _type, std::string _value, std::string _info) :
    on_close_callback(_on_close_callback),
    name(_name),
    type(_type),
    value(_value),
    info(_info)
{
    init_members();

    Glib::ustring name_ustring = name;
    Glib::ustring type_ustring = type;
    Glib::ustring value_ustring = value;
    Glib::ustring info_ustring = info;

    //Create fields with given values - TODO field types depend on input type, listener for type change
    name_entry = Gtk::manage(new Gtk::Entry());
    name_entry.set_text(name_ustring);
    type_entry = Gtk::manage(new Gtk::Entry());
    type_entry.set_text(type_ustring);
    value_entry = Gtk::manage(new Gtk::Entry());
    value_entry.set_text(value_ustring);
    info_entry = Gtk::manage(new Gtk::Entry());
    info_entry.set_text(info_ustring);

    params_create_values_grid->attach(*name_entry, 1, 0);
    params_create_values_grid->attach(*type_entry, 1, 1);
    params_create_values_grid->attach(*value_entry, 1, 2);
    params_create_values_grid->attach(*info_entry, 1, 3);
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
    on_close_callback("","","","");
}

void ParamsCreateView::on_add() {
    window->close();
    on_close_callback("test","test","test","test");
}
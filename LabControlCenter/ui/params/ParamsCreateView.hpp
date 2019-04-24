#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <cassert>
#include <string>
#include <functional>
#include <iostream>

class ParamsCreateView {
private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Window* window;
    Gtk::Widget* parent;
    Gtk::Box* params_create_box;
    Gtk::ButtonBox* params_create_buttons;
    Gtk::Button* params_create_abort_button;
    Gtk::Button* params_create_add_button;
    Gtk::Grid* params_create_values_grid;

    //Custom fields that allow to edit data
    Gtk::Entry* name_entry;
    Gtk::Entry* type_entry;
    Gtk::Entry* value_entry;
    Gtk::Entry* info_entry;

    //Function used by both constructors
    void init_members();
    void create_inputs(std::string name, std::string type, std::string value, std::string info);

    //Callback function on close (must always be called!)
    std::function<void(ParameterWithDescription, bool)> on_close_callback;

    //Callback functions for buttons
    void on_abort();
    void on_add();

    //Helper conversion functions - also used to check data correctness? TODO
    bool string_to_bool(std::string str); //Returns false if the argument is invalid
    bool string_to_int(std::string str);
    bool string_to_double(std::string str);
    bool string_to_int_vector(std::string str);
    bool string_to_double_vector(std::string str);
public:
    ParamsCreateView(std::function<void(ParameterWithDescription, bool)> on_close_callback);
    ParamsCreateView(std::function<void(ParameterWithDescription, bool)> on_close_callback, std::string _name, std::string _type, std::string _value, std::string _info);
};
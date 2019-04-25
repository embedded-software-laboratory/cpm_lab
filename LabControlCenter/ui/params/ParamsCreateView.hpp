#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <cassert>
#include <string>
#include <functional>
#include <iostream>
#include "ParameterWithDescription.hpp"

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
    Gtk::ComboBoxText* type_entry;
    Gtk::Entry* value_entry = nullptr;
    Gtk::Switch* value_switch = nullptr;
    Gtk::Entry* info_entry;

    //Function used by both constructors
    void init_members();
    void create_inputs();
    void on_type_changed(); //Changes the value input

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

    //Parameter object that is created or modified
    ParameterWithDescription param;
    int float_precision; //Precision of floats
public:
    ParamsCreateView(std::function<void(ParameterWithDescription, bool)> on_close_callback, int float_precision);
    ParamsCreateView(std::function<void(ParameterWithDescription, bool)> on_close_callback, ParameterWithDescription param, int float_precision);
};
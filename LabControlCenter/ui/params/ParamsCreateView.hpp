#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <locale>
#include <string>
#include <vector>
#include "ParameterWithDescription.hpp"

#define MAX_INT32_SYMBOL (2147483647l)
#define MIN_INT32_SYMBOL (-2147483648l)

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
    Gtk::Entry value_entry;
    Gtk::Switch value_switch;
    Gtk::Entry* info_entry;

    //Function used by both constructors
    void init_members();
    void create_inputs();
    void on_type_changed(); //Changes the value input

    //Callback function on close (must always be called!)
    std::function<void(ParameterWithDescription, bool)> on_close_callback;
    std::function<bool(std::string)> check_param_exists;

    //Callback functions for buttons
    void on_abort();
    void on_add();

    //Callback functions for signals emitted by entries (when the user changes the input)
    void on_value_entry_changed();
    void on_name_entry_changed();

    //Helper conversion functions - also used to check data correctness; return false if the argument is invalid
    bool string_to_int(std::string str, int32_t& value);
    bool string_to_double(std::string str, double& value);
    bool string_to_int_vector(std::string str, std::vector<int32_t>& value);
    bool string_to_double_vector(std::string str, std::vector<double>& value);

    //Parameter object that is created or modified
    ParameterWithDescription param;
    bool is_edit_window; //Behaviour different if new param created or old param edited
    int float_precision; //Precision of floats
public:
    ParamsCreateView(std::function<void(ParameterWithDescription, bool)> on_close_callback, std::function<bool(std::string)> check_param_exists, int float_precision);
    ParamsCreateView(std::function<void(ParameterWithDescription, bool)> on_close_callback, std::function<bool(std::string)> check_param_exists, ParameterWithDescription param, int float_precision);
};
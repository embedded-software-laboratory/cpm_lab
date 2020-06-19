// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <locale>
#include <string>
#include <vector>
#include "ParameterWithDescription.hpp"

#define MAX_INT32_SYMBOL (2147483647l)
#define MIN_INT32_SYMBOL (-2147483648l)

/**
 * \brief This class creates a window where params can be changed, deleted or created
 */
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
    void init_members(Gtk::Window& parent);
    void create_inputs();
    void on_type_changed(); //Changes the value input

    //Callback function on close (must always be called!)
    std::function<void(ParameterWithDescription, bool)> on_close_callback;
    std::function<bool(std::string)> check_param_exists;

    //Key events - act depending on which button was released
    bool handle_button_released(GdkEventKey* event);

    //Callback functions for buttons / delete event
    bool on_delete(GdkEventAny* any_event);
    void on_abort();
    void on_add();
    bool called_callback = false;

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
    ParamsCreateView(Gtk::Window& main_window, std::function<void(ParameterWithDescription, bool)> on_close_callback, std::function<bool(std::string)> check_param_exists, int float_precision);
    ParamsCreateView(Gtk::Window& main_window, std::function<void(ParameterWithDescription, bool)> on_close_callback, std::function<bool(std::string)> check_param_exists, ParameterWithDescription param, int float_precision);
};
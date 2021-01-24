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
#define MAX_UINT64_SYMBOL (18446744073709551615ull)

/**
 * \brief LCC UI class that creates a window where params can be changed, deleted or created
 * \ingroup lcc_ui
 */
class ParamsCreateView {
private:
    //! GTK Builder for UI
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    //! Parent window, used to register key handling etc.
    Gtk::Window* window;
    //! Also parent window, deprecated
    Gtk::Widget* parent;
    //! Contains buttons and parameter description / entries
    Gtk::Box* params_create_box;
    //! Box that contains the buttons to add a param or abort param creation
    Gtk::ButtonBox* params_create_buttons;
    //! Button to abort param creation / edit
    Gtk::Button* params_create_abort_button;
    //! Button to add new or save edited param, if it does not already exist
    Gtk::Button* params_create_add_button;
    //! Contains labels and entries for param creation
    Gtk::Grid* params_create_values_grid;

    //Custom fields that allow to edit data
    //! Entry for parameter name
    Gtk::Entry* name_entry;
    //! Entry for parameter type
    Gtk::ComboBoxText* type_entry;
    //! Entry for parameter value
    Gtk::Entry value_entry;
    //! Entry for parameter value type
    Gtk::Switch value_switch;
    //! Entry for parameter description
    Gtk::Entry* info_entry;

    /**
     * \brief Init function used by both constructors, creates the param window w.r.t. a parent window
     * \param parent The parent window of the param create window
     */
    void init_members(Gtk::Window& parent);

    /**
     * \brief Create empty fields or insert pre-existing parameter values in the fields from the paremeter given in the constructor (name, type, ...)
     */
    void create_inputs();

    /**
     * \brief Changes the value input between boolean switch and entry, depending on the selected parameter type
     */
    void on_type_changed(); 

    //! Callback function on close (must always be called!). Gives feedback to the window creator - new parameter values and if these values should be stored or ignored
    std::function<void(ParameterWithDescription, bool)> on_close_callback;
    //! Allows to check if a parameter already exist, so that a new parameter with the same name cannot be created (instead, the user would have to edit the existing parameter then)
    std::function<bool(std::string)> check_param_exists;

    /**
     * \brief Key events - act depending on which button was released (abort on ESC, save on ENTER)
     * \param event GTK key event
     */
    bool handle_button_released(GdkEventKey* event);

    /**
     * \brief Callback function for delete event / when the window is closed.
     * Do not save changes in that case (set second parameter in on_close_callback to false).
     * \param any_event Ignored, but part of the delete event handler
     */
    bool on_delete(GdkEventAny* any_event);

    /**
     * \brief Callback function for abort button / called in case ESC is released.
     * Calls window->close(), thus triggering on_delete
     */
    void on_abort();

    /**
     * \brief Callback function for add button / ENTER key.
     * The set parameter values are read and stored in a param object.
     * The param is not saved (set second parameter in on_close_callback to false) only 
     * if an existing parameter that was not supposed to be added when the window was created
     * with the same name already exists.
     * 
     * Either closes the window - another callback in on_delete is prevented by a called_callback - 
     * or marks entries that lead to errors:
     * 
     * - Invalid values: value_entry is highlighted
     * 
     * - Invalid name: value_name is highlighted
     */
    void on_add();

    /**
     * \brief To remember if on_close_callback was already called. 
     * This is relevant because on_delete handles every case where a parameter is not stored. 
     * In case a parameter is supposed to be stored, the callback in the delete handler must not be called
     * again when the window gets destroyed.
     */
    bool called_callback = false;

    /**
     * \brief In on_add, a wrong entry for the value might have lead to this field being highlighted. 
     * This style change is removed as soon as the entry is changed.
     */
    void on_value_entry_changed();
    /**
     * \brief In on_add, a wrong entry for the name might have lead to this field being highlighted. 
     * This style change is removed as soon as the entry is changed.
     */
    void on_name_entry_changed();

    /**
     * \brief Helper conversion function - also used to check data correctness; return false if the argument is invalid
     * \param str The input value to be converted
     * \param value The output value, as result of the conversion
     * \return True if conversion worked, else false
     */
    bool string_to_int(std::string str, int32_t& value);
    /**
     * \brief Helper conversion function - also used to check data correctness; return false if the argument is invalid
     * \param str The input value to be converted
     * \param value The output value, as result of the conversion
     * \return True if conversion worked, else false
     */
    bool string_to_uint64_t(std::string str, uint64_t& value);
    /**
     * \brief Helper conversion function - also used to check data correctness; return false if the argument is invalid
     * \param str The input value to be converted
     * \param value The output value, as result of the conversion
     * \return True if conversion worked, else false
     */
    bool string_to_double(std::string str, double& value);
    /**
     * \brief Helper conversion function - also used to check data correctness; return false if the argument is invalid
     * \param str The input value to be converted
     * \param value The output value, as result of the conversion
     * \return True if conversion worked, else false
     */
    bool string_to_int_vector(std::string str, std::vector<int32_t>& value);
    /**
     * \brief Helper conversion function - also used to check data correctness; return false if the argument is invalid
     * \param str The input value to be converted
     * \param value The output value, as result of the conversion
     * \return True if conversion worked, else false
     */
    bool string_to_double_vector(std::string str, std::vector<double>& value);

    //! Parameter object that is created or modified
    ParameterWithDescription param;
    //! The behaviour different if a new param is created or an old param is edited (in case of edit: It is allowed to store a parameter with the same name, as this parameter is edited)
    bool is_edit_window;
    //! Precision of floats
    int float_precision;
public:
    /**
     * \brief Creates a parameter creation window, where the user can create a new parameter, that gets returned with on_close_callback. Not suitable for editing parameters.
     * \param main_window The "reference" window required to have an "attachment point" for creating the param create view window
     * \param on_close_callback Callback function that is called when the window is closed: (New parameter values, boolean - if true, the parameter should be saved, else it must be ignored)
     * \param check_param_exists Callback function that should allow the param window to check if a parameter with the same name, given as string, already exists (true if it does)
     * \param float_precision Precision of floats
     */
    ParamsCreateView(Gtk::Window& main_window, std::function<void(ParameterWithDescription, bool)> on_close_callback, std::function<bool(std::string)> check_param_exists, int float_precision);

    /**
     * \brief Creates a parameter edit window, where the user can edit an existing parameter, that gets returned with on_close_callback. Not suitable for creating new parameters.
     * \param main_window The "reference" window required to have an "attachment point" for creating the param create view window
     * \param on_close_callback Callback function that is called when the window is closed: (Edited parameter values, boolean - if true, the parameter should be saved, else it must be ignored)
     * \param check_param_exists Callback function that should allow the param window to check if a parameter with the same name, given as string, already exists (true if it does)
     * \param param The current value of the parameter to be edited
     * \param float_precision Precision of floats
     */
    ParamsCreateView(Gtk::Window& main_window, std::function<void(ParameterWithDescription, bool)> on_close_callback, std::function<bool(std::string)> check_param_exists, ParameterWithDescription param, int float_precision);
};
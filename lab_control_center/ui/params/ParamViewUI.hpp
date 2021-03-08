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

#include "defaults.hpp"
#include "ParamModelRecord.hpp"
#include "ParamsCreateView.hpp"
#include "ParameterStorage.hpp"
#include "ParameterWithDescription.hpp"
#include "cpm/Logging.hpp"

#include "LCCErrorLogger.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <gtkmm/liststore.h>
#include <cassert>
#include <string>
#include <atomic>
#include <memory>
#include <sstream>
#include <functional>

/**
 * \class ParamViewUI
 * \brief UI class of the LCC to show and change / delete parameters that are sent to other participants via DDS (mostly for initialization)
 * \ingroup lcc_ui
 */
class ParamViewUI {
private:
    //! Storage for all parameters
    std::shared_ptr<ParameterStorage> parameter_storage;

    //! Error dialog if loading a parameter fails
    std::shared_ptr<Gtk::MessageDialog> error_dialog;

    //! GTK UI builder
    Glib::RefPtr<Gtk::Builder> params_builder;

    //! Parent widget, can be used to insert the param view into another widget with get_parent()
    Gtk::Widget* parent;

    //Top box: search bar and filter
    //Gtk::FlowBox* parameters_flow_top;
    //Gtk::SearchEntry* parameters_search;
    //Gtk::Box* parameters_box_filter;
    //Gtk::Label* parameters_filter_description;
    //Gtk::ComboBoxText* parameters_filter;

    //! Box: Contains parameter list, scrollable window
    Gtk::ScrolledWindow* parameters_list_scroll_window;
    //! Actual parameter list shown in the UI
    Gtk::TreeView* parameters_list_tree;

    //Bottom box: Buttons like edit, delete, new
    //! Contains edit, delete and create buttons
    Gtk::FlowBox* parameters_box_buttons;
    //! Contains delete button
    Gtk::FlowBoxChild* parameters_box_delete;
    //! Parameter delete button (for the current selection)
    Gtk::Button* parameters_button_delete;
    //! Contains edit button
    Gtk::FlowBoxChild* parameters_box_edit;
    //! Parameter edit button (for the current selection)
    Gtk::Button* parameters_button_edit;
    //! Contains create button
    Gtk::FlowBoxChild* parameters_box_create;
    //! Parameter creation button
    Gtk::Button* parameters_button_create;

    //! TreeView Layout for parameters (defines columns)
    ParamModelRecord model_record;
    //! TreeView storage (for currently visible parameters)
    Glib::RefPtr<Gtk::ListStore> parameter_list_storage;

    //! Edit / create window that is created whenever a parameter is edited or created
    std::shared_ptr<ParamsCreateView> create_window;
    //! If true, another window is opened that might modify parameters, thus other modification is not allowed / the parameter view cannot be used
    std::atomic<bool> parameter_view_unchangeable;
    //! To check whether a new parameter was added or if a selected parameter was modified when the result of the create window needs to be stored
    bool create_window_open = false;

    //! Function to get the main window, which is required to attach the param create window to it
    std::function<Gtk::Window&()> get_main_window;

    /**
     * \brief First reset the currently shown list of parameters. 
     * Then read all data from parameter storage and put it in the parameter Treeview / UI.
     */
    void read_storage_data();

    /**
     * \brief Get the currently selected row, if one exists, else return false.
     * Store the name of the selected parameter in name.
     * The row is selected by the user in the UI.
     * \param name The currently selected parameter.
     */
    bool get_selected_row(std::string &name);
    
    /**
     * \brief Callback function: Delete the row selected by the user if the delete button was clicked or released on the keyboard
     */
    void delete_selected_row();

    /**
     * \brief Callback function: When the create button is pressed, open a param create window
     */
    void open_param_create_window();

    /**
     * \brief Callback function: When the edit button is pressed, a currently selected entry is double clicked 
     * or ENTER is released when an entry is selected, open a param edit window.
     */
    void open_param_edit_window();

    //Key / mouse events
    /**
     * \brief Key event. Edit a selected param if RETURN is released, delete one if DELETE is released.
     * \param event The GTK key release event
     */
    bool handle_button_released(GdkEventKey* event);

    /**
     * \brief Mouse event. Edit a selected param if it is double-clicked
     * \param button_event The GTK mouse button event
     */
    bool handle_mouse_event(GdkEventButton* button_event);

    //Callbacks for ParamsCreateView
    /**
     * \brief Callback for ParamsCreateView: Allow to only create another create window when the former window was closed.
     * Handles callback for close and for create operations.
     * Parameter is stored / edited if valid_parameter is true.
     * Variables like create_window_open are used to determine if the parameter already exists and must be edited,
     * or if a create window called the callback and thus a new parameter must be created.
     * \param param Parameter returned by the ParamsCreateView window
     * \param valid_parameter True if the parameter should be stored, else false
     */
    void window_on_close_callback(ParameterWithDescription param, bool valid_parameter);

    /**
     * \brief Callback for ParamsCreateView: Access the parameter storage, return true if a parameter with the given name already exists,
     * else false.
     * \param name The parameter name to check
     */
    bool check_param_exists_callback(std::string name);

    //! Precision of floats
    int float_precision;
public:
    /**
     * \brief Constructor, creates a ParamViewUI object with access to the parameter storage, which is shown to the user and can be edited by them.
     * \param parameter_storage The data storage that needs to be accessed from the UI to present data and store/modify it if the user does so
     * \param float_precision precision of floats shown by the UI
     */
    ParamViewUI(std::shared_ptr<ParameterStorage> parameter_storage, int float_precision);

    /**
     * \brief Returns the parent widget of the ParamViewUI object, which can be used to put the UI in another UI element
     */
    Gtk::Widget* get_parent();

    //Callbacks for button presses on menu items
    /**
     * \brief Callback that can be registered for a param reload button.
     * The parameter storage reads the currently selected file and the UI shows the read values.
     * Values that were not saved are discarded.
     */
    void params_reload_handler();

    /**
     * \brief Callback that can be registered for a param save button.
     * The values currently stored in the UI and parameter storage, 
     * potentially modified by the user, are stored in the currently selected file.
     */
    void params_save_handler();

    /**
     * \brief Callback that can be registered for a param save as button.
     * The values currently stored in the UI and parameter storage, 
     * potentially modified by the user, are stored in the selected file.
     * \param filename The name of the file in which the parameter values should be stored.
     */
    void params_save_as_handler(std::string filename);

    /**
     * \brief Callback that can be registered for a param load button.
     * The parameter storage reads the selected file and the UI shows the read values.
     * Values that were not saved are discarded.
     * \param filename The (new) param file to load.
     */
    void params_load_file_handler(std::string filename);
    // void params_load_multiple_files_handler();
    // void params_load_params_handler();

    //Calls to make the UI (in)sensitive during changes
    /**
     * \brief Makes the UI sensitive / interaction with the user is possible.
     */
    void make_sensitive();
    /**
     * \brief Makes the UI insensitive / interaction with the user is not possible.
     */
    void make_insensitive();

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "required" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);
};
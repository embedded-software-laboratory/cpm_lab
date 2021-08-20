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
#include <experimental/filesystem> //Used instead of std::filesystem, because some compilers still seem to be outdated
#include <fstream>
#include <functional>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <vector>
//For storing last chosen file(s)
#include "FileDialogPaths.hpp"

/**
 * \class FileChooserUI
 * \brief A UI class for a file chooser dialog
 * \ingroup lcc_ui
 */
class FileChooserUI {
public:
    /**
     * \struct Filter
     * \brief Filter struct for defining Filters for the file chooser dialog (such as: only show files of type .m, .exe etc...)
     * \ingroup lcc_ui
     */
    struct Filter 
    {
        //! Filter name in the file chooser window
        std::string name;
        //! Filter definition in mime types, cannot be combined with patterns
        std::vector<std::string> mime_filter_types;
        //! Filter defintion using patterns, cannot be combined with mime types
        std::vector<std::string> pattern_filter_types;
    };

    ~FileChooserUI() {
        std::cout << "!!! --- FileChooserUI destructor" << std::endl;
    }

private:
    //! GTK UI Builder for the file chooser
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    //! Main window
    Gtk::Window* window;
    //! File chooser dialog UI
    Gtk::FileChooserDialog* file_chooser_dialog;
    //! Abort button in the file chooser
    Gtk::Button* button_abort;
    //! Load file button in the file chooser
    Gtk::Button* button_load;

    /**
     * \brief Init function, shared by the different constructors
     * \param parent Parent window to be set for this object, to have a reference to a top window when a dialog is created
     * \param filters Filters to be shown in the file chooser. Only files of that type can be selected
     * \param opened_path Optional. Tells the file chooser where to start (if the path exists), else ignored
     */
    void init(Gtk::Window& parent, std::vector<Filter> filters, std::string opened_path);

    //! Callback function called on file chooser close (must always be called!)
    std::function<void(std::string, bool)> on_close_callback;

    //Callback functions for buttons and delete event
    /**
     * \brief Callback function for delete event of the file chooser, calls on_close_callback
     * \param any_event Ignored event that caused the delete
     */
    bool on_delete(GdkEventAny* any_event);
    /**
     * \brief Callback function for abort event of the file chooser, closes the window
     */
    void on_abort();
    /**
     * \brief Callback function for load event of the file chooser.
     * When a file gets loaded, the window is closed and on_close_callback gets called.
     */
    void on_load();

    //Key events - act depending on which button was released
    /**
     * \brief UI key event to react to Enter and Escape Key with loading and aborting
     * \param event UI key event
     */
    bool handle_button_released(GdkEventKey* event);
    /**
     * \brief UI mouse event to load a file when it is double clicked
     * \param event UI Button event
     */
    bool handle_double_click(GdkEventButton* event);
    //! Boolean to remember if a callback was called before, currently not properly used
    bool called_callback = false;

    //! Name for the config file, to remember the last opened file for a specific use-case, e.g. for parameters, scripts, commonroad scenarios etc.
    std::string config_name;
public:
    /**
     * \brief Constructor for a file chooser dialog 
     * \param parent Window parent of the current window, settings this prevents errors / warnings
     * \param on_close_callback Callback function to get the string of the chosen file and if a file was chosen
     * \param config_name Optional. In a configuration file, previous file locations are stored, for the convenience of the user. The config name, if not default, can be used to remember the last file for this specific use-case (e.g. for parameters).
     * \param opened_path Optional. Tells the file chooser where to start (if the path exists), else ignored
     */
    FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> on_close_callback, std::string config_name = "default", std::string opened_path = "");

    /**
     * \brief Constructor for a file chooser dialog that allows to set filters (bottom right, which items are shown besides folders)
     * For each filter name in filter_name, one or more filters are set using filter_type (first entry in filter name corresponds to first entry in filter_type etc)
     * \param parent Window parent of the current window, settings this prevents errors / warnings
     * \param on_close_callback Callback function to get the string of the chosen file and if a file was chosen
     * \param filters Filters to set for choosing a file, e.g. only YAML files are shown
     * \param config_name Optional. In a configuration file, previous file locations are stored, for the convenience of the user. The config name, if not default, can be used to remember the last file for this specific use-case (e.g. for parameters).
     * \param opened_path Optional. Tells the file chooser where to start (if the path exists), else ignored
     */
    FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> on_close_callback, std::vector<Filter> filters, std::string config_name = "default", std::string opened_path = "");

    /**
     * \brief Returns the previously selected path of last program execution
     * \param config_name In a configuration file, previous file locations are stored, for the convenience of the user. The config name, if not default, can be used to remember the last file for this specific use-case (e.g. for parameters).
     */
    static std::string get_last_execution_path(std::string config_name = "default");
};
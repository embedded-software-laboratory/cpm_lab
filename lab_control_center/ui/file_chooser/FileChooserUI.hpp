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
#include <fstream>
#include <functional>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

class FileChooserUI {
public:
    //Filter struct for defining Filters for the file chooser dialog (such as: only show files of type .m, .exe etc...)
    struct Filter 
    {
        std::string name;
        std::vector<std::string> mime_filter_types;
        std::vector<std::string> pattern_filter_types;
    };

private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Window* window;
    Gtk::FileChooserDialog* file_chooser_dialog;
    Gtk::Button* button_abort;
    Gtk::Button* button_load;

    void init(Gtk::Window& parent, std::vector<Filter> filters);

    //Callback function on close (must always be called!)
    std::function<void(std::string, bool)> on_close_callback;

    //Callback functions for buttons and delete event
    bool on_delete(GdkEventAny* any_event);
    void on_abort();
    void on_load();

    //Key events - act depending on which button was released
    bool handle_button_released(GdkEventKey* event);
    bool handle_double_click(GdkEventButton* event);
    bool called_callback = false;

    //Remember last opened file (also in between program executions)
    std::string previous_file = "./";
    static const std::string default_load_path;
    std::string config_location;
public:
    /**
     * \brief Constructor for a file chooser dialog 
     * \param parent Window parent of the current window, settings this prevents errors / warnings
     * \param on_close_callback Callback function to get the string of the chosen file and if a file was chosen
     * \param config_file Configuration file where previous file locations are stored, for the convenience of the user (set relative filepath as well, e.g. "./xy.config")
     */
    FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> on_close_callback, std::string config_file = "./file_dialog_open_config.config");

    /**
     * \brief Constructor for a file chooser dialog that allows to set filters (bottom right, which items are shown besides folders)
     * For each filter name in filter_name, one or more filters are set using filter_type (first entry in filter name corresponds to first entry in filter_type etc)
     * \param parent Window parent of the current window, settings this prevents errors / warnings
     * \param on_close_callback Callback function to get the string of the chosen file and if a file was chosen
     * \param filters Filters to set for choosing a file, e.g. only YAML files are shown
     * \param config_file Configuration file where previous file locations are stored, for the convenience of the user (set relative filepath as well, e.g. "./xy.config")
     */
    FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> on_close_callback, std::vector<Filter> filters, std::string config_file = "./file_dialog_open_config.config");

    /**
     * \brief Returns the previously selected path of last program execution
     * \param config_file Configuration file where previous file locations are stored, for the convenience of the user (set relative filepath as well, e.g. "./xy.config")
     */
    static std::string get_last_execution_path(std::string config_file = "./file_dialog_open_config.config");
};
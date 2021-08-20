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
#include <string>
#include <vector>
//For remembering the last saved file
#include "FileDialogPaths.hpp"

/**
 * \class FileSaverUI
 * \brief A UI class for a file saver dialog
 * \ingroup lcc_ui
 */
class FileSaverUI {
private:
    //! GTK UI builder
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    //! Window of the dialog
    Gtk::Window* window;
    //! GTK file saver dialog
    Gtk::FileChooserDialog* file_saver_dialog;
    //! File saver dialog abort button
    Gtk::Button* button_abort;
    //! File saver dialog save button
    Gtk::Button* button_save;

    //! Callback function on close (must always be called!)
    std::function<void(std::string, bool)> on_close_callback;

    //Callback functions for buttons and delete event
    /**
     * \brief Callback function that gets called when the dialog gets deleted
     * \param any_event Ignored event 
     */
    bool on_delete(GdkEventAny* any_event);
    /**
     * \brief Callback function that gets called when the file saving is aborted
     */
    void on_abort();
    /**
     * \brief Callback function that gets called when the file is supposed to be saved
     */
    void on_save();

    //Key events - act depending on which button was released
    /**
     * \brief React to a key release event: Enter for saving, Escape for quitting the dialog
     * \param event Key event to react to
     */
    bool handle_button_released(GdkEventKey* event);
    //! If the callback was called before
    bool called_callback = false;

    //! Name for the config file, to remember the last opened file for a specific use-case, e.g. for parameters, scripts, commonroad scenarios etc.
    std::string config_name;
public:
    /**
     * \brief Constructor
     * \param parent Parent window, a reference is required to be able to properly handle the GTK Dialog
     * \param on_close_callback Callback function the be called when the saver is closed, to get the relevant information (file path, if a file is supposed to be saved)
     * \param config_name In a configuration file, previous file locations are stored, for the convenience of the user. The config name, if not default, can be used to remember the last file for this specific use-case (e.g. for parameters).
     */
    FileSaverUI(Gtk::Window& parent, std::function<void(std::string, bool)> on_close_callback, std::string config_name = "default");

    ~FileSaverUI() {
        std::cout << "!!! --- FileSaverUI destructor" << std::endl;
    }
};
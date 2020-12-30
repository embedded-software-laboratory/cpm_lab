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

/**
 * \class FileSaverUI
 * \brief A UI class for a file saver dialog
 * \ingroup lcc_ui
 */
class FileSaverUI {
private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Window* window;
    Gtk::FileChooserDialog* file_saver_dialog;
    Gtk::Button* button_abort;
    Gtk::Button* button_save;

    //Callback function on close (must always be called!)
    std::function<void(std::string, bool)> on_close_callback;

    //Callback functions for buttons and delete event
    bool on_delete(GdkEventAny* any_event);
    void on_abort();
    void on_save();

    //Key events - act depending on which button was released
    bool handle_button_released(GdkEventKey* event);
    bool called_callback = false;

    //Remember last opened file (also in between program executions)
    static std::string previous_file;
    static bool file_config_loaded;
    static const std::string file_dialog_config_location;
public:
    FileSaverUI(Gtk::Window& parent, std::function<void(std::string, bool)> on_close_callback);
};
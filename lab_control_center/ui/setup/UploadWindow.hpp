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
#include <cassert>
#include <sstream>
#include <string>
#include <vector>

#include <iostream>

/**
 * \brief This window shows a message during the upload process to the HLCs
 * Important: This window is also shown when no upload is taking place, to tell the user that the upload failed because their vehicle choice / the HLC count was zero
 * \ingroup lcc_ui
 */
class UploadWindow {
private:
    //! GTK UI Builder
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    //! GTK (upload) window
    Gtk::Window* upload_window;
    //! GTK Label for text / information / error messages shown in the upload window
    Gtk::Label* label_upload;
public:
    /**
     * \brief Constructor for an upload window object. Displays the window immediately.
     * \param parent Parent window of the upload window
     * \param vehicle_ids Vehicle IDs, to display matching between vehicle and HLC IDs (first entry matched to first entry etc.)
     * \param hlc_ids HLC IDs, to display matching between vehicle and HLC IDs (first entry matched to first entry etc.)
     */
    UploadWindow(Gtk::Window& parent, std::vector<unsigned int> vehicle_ids, std::vector<uint8_t> hlc_ids);

    ~UploadWindow() {
        std::cout << "!!! --- UploadWindow destructor" << std::endl;
    }

    /**
     * \brief If an upload failed, then show an according message to the user
     * \param msg The error message to append
     */
    void add_error_message(std::string msg);

    /**
     * \brief For custom text that should be shown in the upload window
     * \param text The text to show
     */
    void set_text(std::string text);

    /**
     * \brief Close the upload window. To show a new window, you need to create a new UploadWindow object.
     */
    void close();
};
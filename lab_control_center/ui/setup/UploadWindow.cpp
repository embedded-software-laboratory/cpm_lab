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

#include "UploadWindow.hpp"

/**
 * \file UploadWindow.cpp
 * \ingroup lcc_ui
 */

UploadWindow::UploadWindow(Gtk::Window& parent, std::vector<unsigned int> vehicle_ids, std::vector<uint8_t> hlc_ids)
{
    params_create_builder = Gtk::Builder::create_from_file("ui/setup/upload_window.glade");

    params_create_builder->get_widget("label_upload", label_upload);
    params_create_builder->get_widget("upload_window", upload_window);

    assert(label_upload);
    assert(upload_window);

    //Set parent for dialog window s.t. Gtk does not show warnings
    upload_window->set_transient_for(parent);

    upload_window->set_deletable(true); //No close button

    //Set label text depending on situation - no upload, missing HLC, correct upload
    std::stringstream label_string;
    if (hlc_ids.size() == 0)
    {
        label_string << "INFO: No HLCs are online, deploying locally only...";
    }
    else if (vehicle_ids.size() == 0)
    {
        label_string << "ERROR: No vehicles selected, aborting deployment...";
    }
    else {
        label_string << "Your script is now being uploaded to the following HLCs (associated vehicle IDs in brackets):\n|";
        size_t lower_index = std::min(vehicle_ids.size(), hlc_ids.size());
        for (size_t i = 0; i < lower_index; ++i)
        {
            label_string << static_cast<unsigned int>(hlc_ids.at(i)) << " - (" << vehicle_ids.at(i) << ") | ";
        }
        if (vehicle_ids.size() > hlc_ids.size())
        {
            label_string << "\n\nWARNING: Less HLCs than selected vehicle IDs available, \
		    some vehicles will be deployed locally.";
        }
    }
    label_upload->set_text(label_string.str().c_str());

    upload_window->show();
}

void UploadWindow::close()
{
    upload_window->close();
}

void UploadWindow::add_error_message(std::string msg)
{
    auto previous_text = label_upload->get_text();
    std::stringstream label_string;
    label_string << previous_text.c_str() << std::endl
        << msg;
    label_upload->set_text(label_string.str().c_str());
}

void UploadWindow::set_text(std::string text)
{
    label_upload->set_text(text.c_str());
}

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

#include "FileSaverUI.hpp"

std::string FileSaverUI::previous_file = "./";
const std::string FileSaverUI::file_dialog_config_location = "./file_dialog_save_config.txt";
bool FileSaverUI::file_config_loaded = false;

FileSaverUI::FileSaverUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback) :
    on_close_callback(_on_close_callback)
{
    params_create_builder = Gtk::Builder::create_from_file("ui/file_chooser/FileSaverDialog.glade");

    params_create_builder->get_widget("file_saver_dialog", file_saver_dialog);
    params_create_builder->get_widget("file_saver_dialog", window);
    params_create_builder->get_widget("button_abort", button_abort);
    params_create_builder->get_widget("button_save", button_save);

    assert(file_saver_dialog);
    assert(window);
    assert(button_abort);
    assert(button_save);

    //Set parent for dialog window s.t. Gtk does not show warnings
    window->set_transient_for(parent);

    //Set values so that the other cannot be used until the parameter is set
    window->set_deletable(true); //No close button, user must use "abort" or "add"
    window->show();

    button_abort->signal_clicked().connect(sigc::mem_fun(this, &FileSaverUI::on_abort));
    button_save->signal_clicked().connect(sigc::mem_fun(this, &FileSaverUI::on_save));

    //Set filter
    auto filter_yaml = Gtk::FileFilter::create();
    filter_yaml->set_name("YAML files");
    filter_yaml->add_mime_type("text/yaml");
    file_saver_dialog->add_filter(filter_yaml);

    file_saver_dialog->set_select_multiple(false);

    file_saver_dialog->signal_key_release_event().connect(sigc::mem_fun(this, &FileSaverUI::handle_button_released));
    file_saver_dialog->add_events(Gdk::KEY_RELEASE_MASK);

    //Load filename from previous program execution once
    if (!file_config_loaded)
    {
        file_config_loaded = true;
        std::string path = "";
        std::ifstream input_stream(file_dialog_config_location);
        if (input_stream.good())
        {
            std::getline(input_stream, previous_file);
        }   
        input_stream.close();
    }

    //Open default or recently opened file / folder (not recommended by gtkmm documentation, because they want the user to use the "Recent"-Tab instead)
    //Only do this if this is a yaml file
    std::string end = ".yaml";
    if (previous_file.size() > end.size()) {
        if (previous_file.compare(previous_file.size() - end.size(), end.size(), end) == 0)
        {
            file_saver_dialog->set_filename(previous_file);
        }
    }

    //Listen for delete event - so that callback function is always called properly
    window->signal_delete_event().connect(sigc::mem_fun(this, &FileSaverUI::on_delete));
}

bool FileSaverUI::handle_button_released(GdkEventKey* event) {
    if (event->type == GDK_KEY_RELEASE)
    {
        if(event->keyval == GDK_KEY_Return) {
            on_save();
            return true;
        }
        else if (event->keyval == GDK_KEY_Escape) {
            on_abort();
            return true;
        }
    }
    return false;
}

//Suppress warning for unused parameter (any_event)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

bool FileSaverUI::on_delete(GdkEventAny* any_event) {
    if (!called_callback) {
        on_close_callback("", false); //false -> do not save changes
    }
    return false;
}

#pragma GCC diagnostic pop

void FileSaverUI::on_abort() {
    window->close();
}

void FileSaverUI::on_save() {
    std::string filename = file_saver_dialog->get_current_name();
    std::string end = ".yaml";

    std::cout << filename << std::endl;

    //Check if this is a valid yaml file according to its name
    bool is_yaml;
    if (filename.size() > end.size()) {
        is_yaml = filename.compare(filename.size() - end.size(), end.size(), end) == 0;
    }

    //If not, add .yaml if no "." is part of the file name
    if (filename.find('.') == std::string::npos) {
        filename += ".yaml";
        is_yaml = true;
    }

    if (is_yaml) {
        std::string path = file_saver_dialog->get_current_folder();
        filename = path + "/" + filename;

        //Remember file for next save dialog
        previous_file = filename;

        //Store previous_file for next program execution
        std::ofstream file;
        file.open(file_dialog_config_location, std::ofstream::out | std::ofstream::trunc);
        file << previous_file << std::endl;
        file.close();

        called_callback = true;
        window->close();
        on_close_callback(filename, true);
    }
}
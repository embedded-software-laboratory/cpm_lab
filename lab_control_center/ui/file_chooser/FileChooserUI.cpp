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

#include "FileChooserUI.hpp"

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback, std::string config_file) :
    on_close_callback(_on_close_callback),
    config_location(config_file)
{
    FileChooserUI::Filter yaml_filter;
    yaml_filter.name = "YAML files";
    yaml_filter.mime_filter_types = std::vector<std::string> {{"text/yaml"}};
    init(parent, std::vector<FileChooserUI::Filter> {{yaml_filter}});
}

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback, std::vector<FileChooserUI::Filter> filters, std::string config_file) :
    on_close_callback(_on_close_callback),
    config_location(config_file)
{
    init(parent, filters);
}

void FileChooserUI::init(Gtk::Window& parent, std::vector<FileChooserUI::Filter> filters)
{
    params_create_builder = Gtk::Builder::create_from_file("ui/file_chooser/FileChooserDialog.glade");

    params_create_builder->get_widget("file_chooser_dialog", file_chooser_dialog);
    params_create_builder->get_widget("file_chooser_dialog", window);
    params_create_builder->get_widget("button_abort", button_abort);
    params_create_builder->get_widget("button_load", button_load);

    assert(file_chooser_dialog);
    assert(window);
    assert(button_abort);
    assert(button_load);

    //Set parent for dialog window s.t. Gtk does not show warnings
    window->set_transient_for(parent);

    //Set values so that the other cannot be used until the parameter is set
    window->set_deletable(true); //No close button, user must use "abort" or "add"
    window->show();

    button_abort->signal_clicked().connect(sigc::mem_fun(this, &FileChooserUI::on_abort));
    button_load->signal_clicked().connect(sigc::mem_fun(this, &FileChooserUI::on_load));

    //Set filter (filter name and which types to filter for)
    for (auto filter : filters)
    {
        auto gtk_filter = Gtk::FileFilter::create();
        gtk_filter->set_name(filter.name.c_str());
        
        for (auto filter_type : filter.mime_filter_types)
        {
            gtk_filter->add_mime_type(filter_type.c_str());
        }
        
        for (auto filter_type : filter.pattern_filter_types)
        {
            gtk_filter->add_pattern(filter_type.c_str());
        }

        file_chooser_dialog->add_filter(gtk_filter);
    }

    file_chooser_dialog->set_select_multiple(false);

    file_chooser_dialog->signal_key_release_event().connect(sigc::mem_fun(this, &FileChooserUI::handle_button_released));
    file_chooser_dialog->signal_button_press_event().connect(sigc::mem_fun(this, &FileChooserUI::handle_double_click));
    file_chooser_dialog->add_events(Gdk::KEY_RELEASE_MASK);

    //Load filename from previous program execution once
    if (!file_config_loaded)
    {
        file_config_loaded = true;
        previous_file = get_last_execution_path(config_location);
    }

    //Open default or recently opened file / folder (not recommended by gtkmm documentation, because they want the user to use the "Recent"-Tab instead)
    file_chooser_dialog->set_filename(previous_file);

    //Listen for delete event - so that callback function is always called properly
    window->signal_delete_event().connect(sigc::mem_fun(this, &FileChooserUI::on_delete));
}

std::string FileChooserUI::get_last_execution_path(std::string config_file)
{
    std::string path = "";
    std::ifstream input_stream(config_file);
    if (input_stream.good())
    {
        std::getline(input_stream, path);
    }   
    input_stream.close();

    return path;
}

bool FileChooserUI::handle_button_released(GdkEventKey* event) {
    if (event->type == GDK_KEY_RELEASE)
    {
        if(event->keyval == GDK_KEY_Return) {
            on_load();
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

bool FileChooserUI::handle_double_click(GdkEventButton* event)
{
    if (event->type == GDK_2BUTTON_PRESS)
    {
        // //We need to make sure that the selected file is not a folder
        // bool is_a_file = true;
        // try
        // {
        //     Glib::RefPtr<Gio::File> file = file_chooser_dialog->get_file();
        //     file->read();
        // }
        // catch (Gio::Error err)
        // {
        //     //Error like IS_DIRECTORY is thrown - I could not find any other quick method to check if it is a file or a directory
        //     is_a_file = false;
        // }
        // catch (...)
        // {
        //     //Maybe take further measures here
        //     is_a_file = false;
        // }

        struct stat info;
        bool is_a_file = true;

        if(stat( file_chooser_dialog->get_filename().c_str(), &info ) != 0)
            is_a_file = false; //Cannot be accessed
        else if(info.st_mode & S_IFDIR)
            is_a_file = false; //Is a directory
        
        if (is_a_file)
        {
            on_load();
            return true;
        }
    }
    return false;
}

bool FileChooserUI::on_delete(GdkEventAny* any_event) {
    if (!called_callback) {
        on_close_callback("", false); //false -> do not save changes
    }
    return false;
}

#pragma GCC diagnostic pop

void FileChooserUI::on_abort() {
    window->close();
}

void FileChooserUI::on_load() {
    std::string filename = file_chooser_dialog->get_filename();
    previous_file = filename;

    //Store previous_file for next program execution
    std::ofstream file;
    file.open(config_location, std::ofstream::out | std::ofstream::trunc);
    file << previous_file << std::endl;
    file.close();
    
    window->close();
    on_close_callback(filename, true);
}
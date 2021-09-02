#include "FileChooserUI.hpp"

/**
 * \file FileChooserUI.cpp
 * \ingroup lcc_ui
 */
FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback, std::string _config_name, std::string opened_path) :
    on_close_callback(_on_close_callback),
    config_name(_config_name)
{
    FileChooserUI::Filter yaml_filter;
    yaml_filter.name = "YAML files";
    yaml_filter.mime_filter_types = std::vector<std::string> {{"text/yaml"}};
    init(parent, std::vector<FileChooserUI::Filter> {{yaml_filter}}, opened_path);
}

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback, std::vector<FileChooserUI::Filter> filters, std::string _config_name, std::string opened_path) :
    on_close_callback(_on_close_callback),
    config_name(_config_name)
{
    init(parent, filters, opened_path);
}

void FileChooserUI::init(Gtk::Window& parent, std::vector<FileChooserUI::Filter> filters, std::string opened_path)
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

    //Load filename from previous program execution or set file
    bool load_config_path = true;
    if (opened_path.size() >= 1)
    {
        //Check if path actually exists
        load_config_path = ! (std::experimental::filesystem::exists(opened_path.c_str()));
    }
    //Open default or recently opened file / folder (not recommended by gtkmm documentation, because they want the user to use the "Recent"-Tab instead)
    if (load_config_path)
    {
        file_chooser_dialog->set_filename(get_last_execution_path(config_name));
    }
    else
    {
        file_chooser_dialog->set_filename(opened_path);
    }

    //Listen for delete event - so that callback function is always called properly
    window->signal_delete_event().connect(sigc::mem_fun(this, &FileChooserUI::on_delete));
}

std::string FileChooserUI::get_last_execution_path(std::string config_name)
{
    return FileDialogPaths::Instance().get_last_execution_path(config_name);;
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

    //Store previous file for next program execution
    FileDialogPaths::Instance().store_last_execution_path(filename, config_name);
    
    window->close();
    on_close_callback(filename, true);
}
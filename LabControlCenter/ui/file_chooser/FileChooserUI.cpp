#include "FileChooserUI.hpp"

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback) :
    on_close_callback(_on_close_callback)
{
    FileChooserUI::Filter yaml_filter;
    yaml_filter.name = "YAML files";
    yaml_filter.mime_filter_types = std::vector<std::string> {{"text/yaml"}};
    init(parent, std::vector<FileChooserUI::Filter> {{yaml_filter}});
}

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback, std::vector<FileChooserUI::Filter> filters) :
    on_close_callback(_on_close_callback)
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
    file_chooser_dialog->add_events(Gdk::KEY_RELEASE_MASK);

    //Listen for delete event - so that callback function is always called properly
    window->signal_delete_event().connect(sigc::mem_fun(this, &FileChooserUI::on_delete));
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

bool FileChooserUI::on_delete(GdkEventAny* any_event) {
    if (!called_callback) {
        on_close_callback("", false); //false -> do not save changes
    }
    return false;
}

void FileChooserUI::on_abort() {
    window->close();
}

void FileChooserUI::on_load() {
    std::string filename = file_chooser_dialog->get_filename();
    
    window->close();
    on_close_callback(filename, true);
}
#include "FileChooserUI.hpp"

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback) :
    on_close_callback(_on_close_callback)
{
    std::vector<std::string> filter_name{"YAML files"}; 
    std::vector<std::string> filter_type{"text/yaml"};
    init(parent, filter_name, filter_type);
}

FileChooserUI::FileChooserUI(Gtk::Window& parent, std::function<void(std::string, bool)> _on_close_callback, std::vector<std::string> filter_name, std::vector<std::string> filter_type) :
    on_close_callback(_on_close_callback)
{
    init(parent, filter_name, filter_type);
}

void FileChooserUI::init(Gtk::Window& parent, std::vector<std::string> filter_name, std::vector<std::string> filter_type)
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

    //Set filter
    size_t min_index = std::min(filter_name.size(), filter_type.size());
    for (size_t index = 0; index < min_index; ++index)
    {
        auto filter = Gtk::FileFilter::create();
        filter->set_name(filter_name.at(index).c_str());
        filter->add_mime_type(filter_type.at(index).c_str());
        file_chooser_dialog->add_filter(filter);
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
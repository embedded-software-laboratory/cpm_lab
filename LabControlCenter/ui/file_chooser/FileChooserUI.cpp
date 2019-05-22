#include "FileChooserUI.hpp"

FileChooserUI::FileChooserUI(std::function<void(std::string, bool)> _on_close_callback) :
    on_close_callback(_on_close_callback)
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

    //Set values so that the other cannot be used until the parameter is set
    window->set_deletable(true); //No close button, user must use "abort" or "add"
    window->show();

    button_abort->signal_clicked().connect(sigc::mem_fun(this, &FileChooserUI::on_abort));
    button_load->signal_clicked().connect(sigc::mem_fun(this, &FileChooserUI::on_load));

    //Set filter
    auto filter_yaml = Gtk::FileFilter::create();
    filter_yaml->set_name("YAML files");
    filter_yaml->add_mime_type("text/yaml");
    file_chooser_dialog->add_filter(filter_yaml);

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
    on_close_callback("", false); //false -> do not save changes
    return false;
}

void FileChooserUI::on_abort() {
    window->close();
}

void FileChooserUI::on_load() {
    std::string filename = file_chooser_dialog->get_filename();
    std::string end = ".yaml";

    //Check if this is a valid yaml file according to its name
    bool is_yaml;
    if (filename.size() > end.size()) {
        is_yaml = filename.compare(filename.size() - end.size(), end.size(), end) == 0;
    }

    if (is_yaml) {
        window->close();
        on_close_callback(filename, true);
    }
}
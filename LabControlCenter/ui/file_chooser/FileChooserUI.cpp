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
    window->set_deletable(false); //No close button, user must use "abort" or "add"
    window->show();

    button_abort->signal_clicked().connect(sigc::mem_fun(this, &FileChooserUI::on_abort));
    button_load->signal_clicked().connect(sigc::mem_fun(this, &FileChooserUI::on_load));
}

void FileChooserUI::on_abort() {
    window->close();
    on_close_callback("", false); //false -> do not save changes
}

void FileChooserUI::on_load() {
    window->close();
    on_close_callback("", false); //TODO
}
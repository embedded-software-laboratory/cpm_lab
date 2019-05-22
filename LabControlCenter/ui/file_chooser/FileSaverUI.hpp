#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <locale>
#include <string>
#include <vector>

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
public:
    FileSaverUI(std::function<void(std::string, bool)> on_close_callback);
};
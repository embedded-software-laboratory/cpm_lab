#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <cassert>
#include <sstream>
#include <string>
#include <vector>

class UploadWindow {
private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Window* upload_window;
    Gtk::Label* label_upload;
public:
    UploadWindow(std::vector<unsigned int> vehicle_ids, std::vector<uint8_t> hlc_ids);

    void close();
};
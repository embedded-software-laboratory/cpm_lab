#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <cassert>
#include <sstream>
#include <string>
#include <vector>

/**
 * \brief This window shows a message during the upload process to the HLCs
 * Important: This window is also shown when no upload is taking place, to tell the user that the upload failed because their vehicle choice / the HLC count was zero
 */
class UploadWindow {
private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Window* upload_window;
    Gtk::Label* label_upload;
public:
    UploadWindow(Gtk::Window& parent, std::vector<unsigned int> vehicle_ids, std::vector<uint8_t> hlc_ids);

    /**
     * \brief If an upload failed, then show an according message to the user
     * \param msg The error message to append
     */
    void add_error_message(std::string msg);

    /**
     * \brief For custom text
     * \param text The text to show
     */
    void set_text(std::string text);

    void close();
};
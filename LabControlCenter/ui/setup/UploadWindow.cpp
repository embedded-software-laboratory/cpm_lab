#include "UploadWindow.hpp"

UploadWindow::UploadWindow(std::vector<unsigned int> vehicle_ids, std::vector<uint8_t> hlc_ids)
{
    params_create_builder = Gtk::Builder::create_from_file("ui/setup/upload_window.glade");

    params_create_builder->get_widget("label_upload", label_upload);
    params_create_builder->get_widget("upload_window", upload_window);

    assert(label_upload);
    assert(upload_window);

    upload_window->set_deletable(true); //No close button

    //Set label text depending on situation - no upload, missing HLC, correct upload
    std::stringstream label_string;
    if (hlc_ids.size() == 0)
    {
        label_string << "ERROR: No HLCs are online, aborting deployment...";
    }
    else if (vehicle_ids.size() == 0)
    {
        label_string << "ERROR: No vehicles selected, aborting deployment...";
    }
    else {
        label_string << "Your script is now being uploaded to the following HLCs (associated vehicle IDs in brackets):\n|";
        size_t lower_index = std::min(vehicle_ids.size(), hlc_ids.size());
        for (size_t i = 0; i < lower_index; ++i)
        {
            label_string << static_cast<unsigned int>(hlc_ids.at(i)) << " - (" << vehicle_ids.at(i) << ") | ";
        }
        if (vehicle_ids.size() > hlc_ids.size())
        {
            label_string << "\n\nWARNING: Less HLCs than selected vehicle IDs available";
        }
    }
    label_upload->set_text(label_string.str().c_str());

    upload_window->show();
}

void UploadWindow::close()
{
    upload_window->close();
}

void UploadWindow::add_error_message(std::string msg)
{
    auto previous_text = label_upload->get_text();
    std::stringstream label_string;
    label_string << previous_text.c_str() << std::endl
        << msg;
    label_upload->set_text(label_string.str().c_str());
}
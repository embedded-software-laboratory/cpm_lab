#include "UploadWindow.hpp"

UploadWindow::UploadWindow(std::vector<unsigned int> vehicle_ids, std::vector<uint8_t> hlc_ids)
{
    params_create_builder = Gtk::Builder::create_from_file("ui/setup/upload_window.glade");

    params_create_builder->get_widget("label_upload", label_upload);
    params_create_builder->get_widget("upload_window", upload_window);

    assert(label_upload);
    assert(upload_window);

    upload_window->set_deletable(true); //No close button

    //Set label text
    std::stringstream label_string;
    label_string << "Your script is now being uploaded to the following HLCs (associated vehicle IDs in brackets):\n|";
    size_t lower_index = std::min(vehicle_ids.size(), hlc_ids.size());
    for (int i = 0; i < lower_index; ++i)
    {
        label_string << static_cast<unsigned int>(hlc_ids.at(i)) << " - (" << vehicle_ids.at(i) << ") | ";
    }
    label_upload->set_text(label_string.str().c_str());

    upload_window->show();
}

void UploadWindow::close()
{
    upload_window->close();
}
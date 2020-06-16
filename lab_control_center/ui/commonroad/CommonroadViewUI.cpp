#include "CommonroadViewUI.hpp"

CommonroadViewUI::CommonroadViewUI
    (
    std::shared_ptr<CommonRoadScenario> _commonroad_scenario
    ) 
    :
    commonroad_scenario(_commonroad_scenario)
{
    builder = Gtk::Builder::create_from_file("ui/commonroad/commonroad.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("commonroad_box", commonroad_box);
    builder->get_widget("commonroad_path", commonroad_path);
    builder->get_widget("entry_lane_width", entry_lane_width);
    builder->get_widget("entry_translate_x", entry_translate_x);
    builder->get_widget("entry_translate_y", entry_translate_y);
    builder->get_widget("button_choose_commonroad", button_choose_commonroad);
    builder->get_widget("button_load_commonroad", button_load_commonroad);
    builder->get_widget("button_apply_transformation", button_apply_transformation);

    assert(parent);
    assert(commonroad_box);
    assert(commonroad_path);
    assert(entry_lane_width);
    assert(entry_translate_x);
    assert(entry_translate_y);
    assert(button_choose_commonroad);
    assert(button_load_commonroad);
    assert(button_apply_transformation);

    //Register button callbacks
    button_choose_commonroad->signal_clicked().connect(sigc::mem_fun(this, &CommonroadViewUI::open_file_explorer));
    button_load_commonroad->signal_clicked().connect(sigc::mem_fun(this, &CommonroadViewUI::load_chosen_file));
    button_apply_transformation->signal_clicked().connect(sigc::mem_fun(this, &CommonroadViewUI::apply_transformation));

    //Also, single transformation values can be applied on a single key press within the entry
    entry_lane_width->signal_key_release_event().connect(sigc::mem_fun(this, &CommonroadViewUI::apply_entry_scale));
    entry_translate_x->signal_key_release_event().connect(sigc::mem_fun(this, &CommonroadViewUI::apply_entry_translate_x));
    entry_translate_y->signal_key_release_event().connect(sigc::mem_fun(this, &CommonroadViewUI::apply_entry_translate_y));

    //Information on transformation on hover
    entry_lane_width->set_tooltip_text("Set min. lane width. <= 0 means no change desired. Also applies w. Return.");
    entry_translate_x->set_tooltip_text("Set x translation. 0 means no change desired. Applied after scale change. Also applies w. Return.");
    entry_translate_y->set_tooltip_text("Set y translation. 0 means no change desired. Applied after scale change. Also applies w. Return.");
    button_apply_transformation->set_tooltip_text("Permanently apply set transformation to coordinate system. Future transformations are applied relative to new coordinate system.");
}

using namespace std::placeholders;
void CommonroadViewUI::open_file_explorer()
{
    //Filter to show only XML files
    FileChooserUI::Filter xml_filter;
    xml_filter.name = "XML";
    xml_filter.pattern_filter_types = std::vector<std::string> {"*.xml"};

    //Only create the window if we can get the main window
    if (get_main_window)
    {
        file_chooser_window = std::make_shared<FileChooserUI>(
            get_main_window(), 
            std::bind(&CommonroadViewUI::file_explorer_callback, this, _1, _2), 
            std::vector<FileChooserUI::Filter> { xml_filter }
        );
    }
    else
    {
        cpm::Logging::Instance().write("%s", "ERROR: Main window reference is missing, cannot create file chooser dialog");
    }
    
}

void CommonroadViewUI::file_explorer_callback(std::string file_string, bool has_file)
{
    if (has_file)
    {
        commonroad_path->set_text(file_string.c_str());

        //Load chosen file - this function is also used for a button callback and thus does not take the file path as a parameter
        load_chosen_file();
    }
}

double CommonroadViewUI::string_to_double(std::string value, double default_value)
{
    //Disregard empty fields / use default then
    if (value.size() > 0)
    {
        try
        {
            return std::stod(value);
        }
        catch(...)
        {
            std::cerr << "Could not translate string to double (transformation string entry : scale or translate_x/_y)" << std::endl;
            return default_value;
        }
    }

    return default_value;
}

void CommonroadViewUI::apply_transformation()
{
    //Get desired lane width and translation
    double lane_width = string_to_double(std::string(entry_lane_width->get_text().c_str()), 0.0);
    double translate_x = string_to_double(std::string(entry_translate_x->get_text().c_str()), 0.0);
    double translate_y = string_to_double(std::string(entry_translate_y->get_text().c_str()), 0.0);

    if (commonroad_scenario)
    {
        commonroad_scenario->transform_coordinate_system(lane_width, translate_x, translate_y);
    }

    entry_lane_width->set_text("0.0");
    entry_translate_x->set_text("0.0");
    entry_translate_y->set_text("0.0");
}

bool CommonroadViewUI::apply_entry_scale(GdkEventKey* event)
{
    if (event->type == GDK_KEY_RELEASE && event->keyval == GDK_KEY_Return)
    {
        //Get desired lane width and translation
        double lane_width = string_to_double(std::string(entry_lane_width->get_text().c_str()), 0.0);

        if (commonroad_scenario)
        {
            commonroad_scenario->transform_coordinate_system(lane_width, 0.0, 0.0);
        }

        entry_lane_width->set_text("0.0");

        return true;
    }
    return false;
}

bool CommonroadViewUI::apply_entry_translate_x(GdkEventKey* event)
{
    if (event->type == GDK_KEY_RELEASE && event->keyval == GDK_KEY_Return)
    {
        //Get desired lane width and translation
        double translate_x = string_to_double(std::string(entry_translate_x->get_text().c_str()), 0.0);

        if (commonroad_scenario)
        {
            commonroad_scenario->transform_coordinate_system(0.0, translate_x, 0.0);
        }

        entry_translate_x->set_text("0.0");

        return true;
    }
    return false;
}

bool CommonroadViewUI::apply_entry_translate_y(GdkEventKey* event)
{
    if (event->type == GDK_KEY_RELEASE && event->keyval == GDK_KEY_Return)
    {
        //Get desired lane width and translation
        double translate_y = string_to_double(std::string(entry_translate_y->get_text().c_str()), 0.0);

        if (commonroad_scenario)
        {
            commonroad_scenario->transform_coordinate_system(0.0, 0.0, translate_y);
        }

        entry_translate_y->set_text("0.0");

        return true;
    }
    return false;
}


void CommonroadViewUI::load_chosen_file()
{
    std::string filepath = std::string(commonroad_path->get_text().c_str());

    try
    {
        commonroad_scenario->load_file(filepath);

        apply_transformation(); //Apply currently set transformation - TODO: might cause flickering if we do this after loading the file
    }
    catch(const std::exception& e)
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "The chosen scenario file could not be loaded / is not spec-conform. Error message is:\n";
        error_msg_stream << e.what();
        if (get_main_window)
        {
            Gtk::MessageDialog load_failed_dialog = Gtk::MessageDialog(
                get_main_window(),
                error_msg_stream.str(),
                false,
                Gtk::MessageType::MESSAGE_INFO,
                Gtk::ButtonsType::BUTTONS_OK,
                true
            );
            load_failed_dialog.run();
        }
        else
        {
            std::cerr << "Could not load error dialog (UI) - main window callback not set for CommonroadViewUI!" << std::endl;
        }
    }
}

void CommonroadViewUI::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}

void CommonroadViewUI::set_sensitive(bool is_sensitive)
{
    parent->set_sensitive(is_sensitive);
    commonroad_box->set_sensitive(is_sensitive);
    commonroad_path->set_sensitive(is_sensitive);
    entry_lane_width->set_sensitive(is_sensitive);
    entry_translate_x->set_sensitive(is_sensitive);
    entry_translate_y->set_sensitive(is_sensitive);
    button_choose_commonroad->set_sensitive(is_sensitive);
    button_load_commonroad->set_sensitive(is_sensitive);
    button_apply_transformation->set_sensitive(is_sensitive);
}

Gtk::Widget* CommonroadViewUI::get_parent()
{
    return parent;
}
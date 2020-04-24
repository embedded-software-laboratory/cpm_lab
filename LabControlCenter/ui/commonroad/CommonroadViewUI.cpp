#include "CommonroadViewUI.hpp"

CommonroadViewUI::CommonroadViewUI
    (
    std::shared_ptr<CommonRoadScenario> _commonroad_scenario,
    unsigned int argc, 
    char *argv[]
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

    assert(parent);
    assert(commonroad_box);
    assert(commonroad_path);
    assert(entry_lane_width);
    assert(entry_translate_x);
    assert(entry_translate_y);
    assert(button_choose_commonroad);
    assert(button_load_commonroad);

    //Register button callbacks
    button_choose_commonroad->signal_clicked().connect(sigc::mem_fun(this, &CommonroadViewUI::open_file_explorer));
    button_load_commonroad->signal_clicked().connect(sigc::mem_fun(this, &CommonroadViewUI::load_chosen_file));
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

void CommonroadViewUI::load_chosen_file()
{
    std::string filepath = std::string(commonroad_path->get_text().c_str());

    //Get desired lane width
    std::string lane_width_text = std::string(entry_lane_width->get_text().c_str());
    double lane_width = 1.0;
    try
    {
        lane_width = std::stod(lane_width_text);
    }
    catch(...)
    {
        std::cerr << "Could not apply desired lane width - not a double" << std::endl;
    }

    commonroad_scenario->load_file(filepath); //TODO: Behaviour on fail; also, maybe set transform already here, or we might accidentally draw in between these two functions with other scale (would cause flickering)
    commonroad_scenario->transform_coordinate_system(lane_width); //TODO: Use data from entries instead; Also TODO: Change on enter in entry after load
}

void CommonroadViewUI::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}

Gtk::Widget* CommonroadViewUI::get_parent()
{
    return parent;
}
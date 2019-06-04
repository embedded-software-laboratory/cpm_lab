#pragma once

#include "defaults.hpp"
#include "ParamModelRecord.hpp"
#include "ParamsCreateView.hpp"
#include "ParameterStorage.hpp"
#include "ParameterWithDescription.hpp"
#include "cpm/Logging.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <gtkmm/liststore.h>
#include <cassert>
#include <string>
#include <atomic>
#include <memory>
#include <sstream>
#include <functional>

class ParamViewUI {
private:
    //Storage for all parameters
    std::shared_ptr<ParameterStorage> parameter_storage;

    Glib::RefPtr<Gtk::Builder> params_builder;

    Gtk::Widget* parent;

    //Top box: search bar and filter
    //Gtk::FlowBox* parameters_flow_top;
    //Gtk::SearchEntry* parameters_search;
    //Gtk::Box* parameters_box_filter;
    //Gtk::Label* parameters_filter_description;
    //Gtk::ComboBoxText* parameters_filter;

    //Middle box: Parameter list
    Gtk::ScrolledWindow* parameters_list_scroll_window;
    Gtk::TreeView* parameters_list_tree;

    //Bottom box: Buttons like edit, delete, new
    Gtk::FlowBox* parameters_box_buttons;
    Gtk::FlowBoxChild* parameters_box_delete;
    Gtk::Button* parameters_button_delete;
    Gtk::FlowBoxChild* parameters_box_edit;
    Gtk::Button* parameters_button_edit;
    Gtk::FlowBoxChild* parameters_box_create;
    Gtk::Button* parameters_button_create;

    //TreeView Layout, Parameters storage
    ParamModelRecord model_record;
    Glib::RefPtr<Gtk::ListStore> parameter_list_storage;

    //Edit / create window
    std::shared_ptr<ParamsCreateView> create_window;
    std::atomic<bool> parameter_view_unchangeable; //If true, another window is opened that might modify parameters, thus other modification is not allowed
    bool create_window_open = false; //To check whether a new parameter was added or if the selected parameter was modified

    //Read all data from parameter storage
    void read_storage_data();

    //Manipulate rows
    bool get_selected_row(std::string &name);
    //"Callback" function: Delete the row selected by the user if the delete button was clicked
    void delete_selected_row();
    //Open edit / create window
    void open_param_create_window();
    void open_param_edit_window();

    //Key events - act depending on which button was released
    bool handle_button_released(GdkEventKey* event);
    bool handle_mouse_event(GdkEventButton* button_event);

    //Callback: Allow to only create another create window when the former window was closed
    //Handles callback for close and for create operations
    void window_on_close_callback(ParameterWithDescription param, bool valid_parameter);
    bool check_param_exists_callback(std::string name);

    //Precision of floats
    int float_precision;
public:
    /**
     * \brief constructor
     * \param parameter_storage The data storage that needs to be accessed from the UI to present data and store/modify it if the user does so
     * \param float_precision precision of floats shown by the UI
     * \param param_server_active_callback callback function that is registered to react to the on/off switch of the param server toggle button
     * /param param_server_active_init initial value of the toggle button
     */
    ParamViewUI(std::shared_ptr<ParameterStorage> parameter_storage, int float_precision);
    Gtk::Widget* get_parent();

    //Callbacks for button presses on menu items
    void params_reload_handler();
    void params_save_handler();
    void params_save_as_handler(std::string filename);
    void params_load_file_handler(std::string filename);
    void params_load_multiple_files_handler();
    void params_load_params_handler();

    //Calls to make the UI (in)sensitive during changes
    void make_sensitive();
    void make_insensitive();
};
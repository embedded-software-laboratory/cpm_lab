#include "ParamViewUI.hpp"

ParamViewUI::ParamViewUI(std::shared_ptr<ParameterStorage> _parameter_storage, int _float_precision, std::function<void(bool)> _param_server_active_callback, bool param_server_active_init) :
    param_server_active_callback(_param_server_active_callback),
    parameter_storage(_parameter_storage),
    float_precision(_float_precision)
{
    try
    {
        params_builder = Gtk::Builder::create_from_file("ui/params/params.glade");
    }
    catch(const Gtk::BuilderError &e)
    {
        std::cerr << "Gtk::BuilderError: " << e.what() << std::endl;
        exit(1);
    }

    params_builder->get_widget("parameters_box", parent);
    // params_builder->get_widget("parameters_flow_top", parameters_flow_top);
    // params_builder->get_widget("parameters_search", parameters_search);
    // params_builder->get_widget("parameters_box_filter", parameters_box_filter);
    // params_builder->get_widget("parameters_filter_description", parameters_filter_description);
    // params_builder->get_widget("parameters_filter", parameters_filter);
    params_builder->get_widget("parameters_list_scroll_window", parameters_list_scroll_window);
    params_builder->get_widget("parameters_list_tree", parameters_list_tree);
    params_builder->get_widget("parameters_box_buttons", parameters_box_buttons);
    params_builder->get_widget("parameters_box_delete", parameters_box_delete);
    params_builder->get_widget("parameters_button_delete", parameters_button_delete);
    params_builder->get_widget("parameters_box_edit", parameters_box_edit);
    params_builder->get_widget("parameters_button_edit", parameters_button_edit);
    params_builder->get_widget("parameters_box_create", parameters_box_create);
    params_builder->get_widget("parameters_button_create", parameters_button_create);
    params_builder->get_widget("parameters_start_server_toggle", parameters_start_server_toggle);

    assert(parent);
    // assert(parameters_flow_top);
    // assert(parameters_search);
    // assert(parameters_box_filter);
    // assert(parameters_filter_description);
    // assert(parameters_filter);
    assert(parameters_list_scroll_window);
    assert(parameters_list_tree);
    assert(parameters_box_buttons);
    assert(parameters_box_delete);
    assert(parameters_button_delete);
    assert(parameters_box_edit);
    assert(parameters_button_edit);
    assert(parameters_box_create);
    assert(parameters_button_create);
    assert(parameters_start_server_toggle);

    //Create data model_record for parameters
    parameter_list_storage = Gtk::ListStore::create(model_record);
    parameters_list_tree->set_model(parameter_list_storage);

    //Use model_record, add it to the view
    parameters_list_tree->append_column("Name", model_record.column_name);
    parameters_list_tree->append_column("Type", model_record.column_type);
    parameters_list_tree->append_column("Value", model_record.column_value);
    parameters_list_tree->append_column("Info", model_record.column_info);

    //Set equal width for all columns
    for (int i = 0; i < 4; ++i) {
        parameters_list_tree->get_column(i)->set_resizable(true);
        parameters_list_tree->get_column(i)->set_min_width(20);
        parameters_list_tree->get_column(i)->set_fixed_width(50);
        parameters_list_tree->get_column(i)->set_expand(true);
    }

    //Read all param data
    read_storage_data();

    //Delete button listener (also keyboard: del)
    parameters_button_delete->signal_clicked().connect(sigc::mem_fun(this, &ParamViewUI::delete_selected_row));
    parameters_list_tree->signal_key_release_event().connect(sigc::mem_fun(this, &ParamViewUI::handle_button_released));
    parameters_list_tree->signal_button_press_event().connect(sigc::mem_fun(this, &ParamViewUI::handle_mouse_event));
    parameters_list_tree->add_events(Gdk::KEY_RELEASE_MASK);

    //Create and edit button listener
    parameter_view_unchangeable.store(false); //Window for creation should only exist once
    parameters_button_create->signal_clicked().connect(sigc::mem_fun(this, &ParamViewUI::open_param_create_window));
    parameters_button_edit->signal_clicked().connect(sigc::mem_fun(this, &ParamViewUI::open_param_edit_window));

    //State change of 'param server active' switch listener
    parameters_start_server_toggle->signal_toggled().connect(sigc::mem_fun(this, &ParamViewUI::on_param_server_active_changed));
    parameters_start_server_toggle->set_tooltip_text("Click to change");
    parameters_start_server_toggle->set_active(param_server_active_init);

    //Save all button listener (save the currently visible configuration to the currently open yaml file)
    //parameters_button_show->signal_clicked().connect(sigc::mem_fun(this, &ParamViewUI::save_configuration));

    //For commas in doubles, also for conversion in ParamsCreateView
    std::setlocale(LC_NUMERIC, "C");
}

void ParamViewUI::read_storage_data() {
    //Clear previously set data
    parameter_list_storage->clear();

    //Read all rows
    for (ParameterWithDescription param : parameter_storage->get_all_parameters()) {
        Gtk::TreeModel::Row row = *(parameter_list_storage->append());

        std::string name;
        std::string type;
        std::string value;
        std::string info;

        ParameterWithDescription::parameter_to_string(param, name, type, value, info, float_precision);

        Glib::ustring name_ustring(name);
        Glib::ustring type_ustring(type);
        Glib::ustring value_ustring(value);
        Glib::ustring info_ustring(info);

        row[model_record.column_name] = name_ustring;
        row[model_record.column_type] = type_ustring;
        row[model_record.column_value] = value_ustring;
        row[model_record.column_info] = info_ustring;
    }
}

Gtk::Widget* ParamViewUI::get_parent() {
    return parent;
}

bool ParamViewUI::get_selected_row(std::string &name) {
    Gtk::TreeModel::iterator iter = parameters_list_tree->get_selection()->get_selected();
    if(iter) //If anything is selected
    {
        Gtk::TreeModel::Row row = *iter;
        Glib::ustring name_ustring(row[model_record.column_name]);
        name = name_ustring;

        return true;
    }
    else return false;
}

bool ParamViewUI::handle_button_released(GdkEventKey* event) {
    if (event->type == GDK_KEY_RELEASE && event->keyval == GDK_KEY_Delete)
    {
        delete_selected_row();
        return true;
    }
    else if (event->type == GDK_KEY_RELEASE && event->keyval == GDK_KEY_Return)
    {
        open_param_edit_window();
        return true;
    }
    return false;
}

bool ParamViewUI::handle_mouse_event(GdkEventButton* button_event) {
    if (button_event->type == GDK_2BUTTON_PRESS)
    {
        open_param_edit_window();
        return true;
    }
    return false;
}

void ParamViewUI::on_param_server_active_changed() {
    bool new_value = parameters_start_server_toggle->get_active();
    
    if (param_server_active_callback) {
        param_server_active_callback(new_value);

        if (new_value) {
            parameters_start_server_toggle->set_label("Toggle parameter server (active)");
        }
        else {
            parameters_start_server_toggle->set_label("Toggle parameter server (inactive)");
        }
    }
    else {
        cpm::Logging::Instance().write("Error: Could not change param server state!");
        parameters_start_server_toggle->set_active(false);
    }

    //UI change after the other changes have been performed
    //parameters_start_server_toggle->set_active(new_value);
    //return false;
}

void ParamViewUI::delete_selected_row() {
    //Parameters cannot be deleted when the edit/create window is opened (or when another parameter has not yet been deleted)
    if (! parameter_view_unchangeable.exchange(true)) {
        Glib::RefPtr<Gtk::TreeSelection> selection = parameters_list_tree->get_selection();
        //Also get the name of the deleted parameter to delete it in the storage object
        std::string name;
        if (selection->get_selected()) {
            Gtk::TreeModel::Row row = *(selection->get_selected());
            Glib::ustring name_ustring(row[model_record.column_name]); //Access name of row
            name = name_ustring;
        }
        std::vector<Gtk::TreeModel::Path> paths = selection->get_selected_rows();

        // convert all of the paths to RowReferences
        std::vector<Gtk::TreeModel::RowReference> rows;
        for (Gtk::TreeModel::Path path : paths)
        {
            rows.push_back(Gtk::TreeModel::RowReference(parameter_list_storage, path));
        }

        // remove the rows from the treemodel
        for (std::vector<Gtk::TreeModel::RowReference>::iterator i = rows.begin(); i != rows.end(); i++)
        {
            Gtk::TreeModel::iterator treeiter = parameter_list_storage->get_iter(i->get_path());
            parameter_list_storage->erase(treeiter);
        }

        //Remove data from parameter storage
        parameter_storage->delete_parameter(name);

        parameter_view_unchangeable.store(false);
    }
}

using namespace std::placeholders;
void ParamViewUI::open_param_create_window() {
    //Get a "lock" for the window if it does not already exist, else ignore the user request
    if(! parameter_view_unchangeable.exchange(true)) {
        make_insensitive();
        create_window_open = true;
        create_window = make_shared<ParamsCreateView>(std::bind(&ParamViewUI::window_on_close_callback, this, _1, _2), std::bind(&ParamViewUI::check_param_exists_callback, this, _1), float_precision);
    } 
}

void ParamViewUI::open_param_edit_window() {
    //Get a "lock" for the window if it does not already exist, else ignore the user request
    if(! parameter_view_unchangeable.exchange(true)) {
        make_insensitive();

        //Get currently selected data
        std::string name;
        std::string type;
        std::string value;
        std::string info;

        //Only create an edit window if a row was selected
        if (get_selected_row(name)) {
            ParameterWithDescription param;
            //Get the parameter
            if (parameter_storage->get_parameter(name, param)) {
                create_window = make_shared<ParamsCreateView>(std::bind(&ParamViewUI::window_on_close_callback, this, _1, _2), std::bind(&ParamViewUI::check_param_exists_callback, this, _1), param, float_precision);
            }
        }
        else {
            make_sensitive();
            parameter_view_unchangeable.store(false);
        }
    } 
}

void ParamViewUI::window_on_close_callback(ParameterWithDescription param, bool valid_parameter) {
    if (valid_parameter) {
        std::string name;
        std::string type;
        std::string value;
        std::string info;

        ParameterWithDescription::parameter_to_string(param, name, type, value, info, float_precision);

        Glib::ustring name_ustring = name;
        Glib::ustring type_ustring = type;
        Glib::ustring value_ustring = value;
        Glib::ustring info_ustring = info;

        Gtk::TreeModel::Row row;

        //If a parameter was modified, get its current row or else create a new row
        Gtk::TreeModel::iterator iter = parameters_list_tree->get_selection()->get_selected();
        if(iter && !create_window_open) //If anything is selected and if param modification window was opened
        {
            row = *iter;
        }
        else if (create_window_open) { //Create a new parameter only if it does not already exist
            row = *(parameter_list_storage->append());
        }

        if (create_window_open || iter) {
            row[model_record.column_name] = name_ustring;
            row[model_record.column_type] = type_ustring;
            row[model_record.column_value] = value_ustring;
            row[model_record.column_info] = info_ustring;

            parameter_storage->set_parameter(name, param);
        }
    }

    //Reset variables so that new windows can be opened etc
    create_window.reset(); //No one manages the shared pointer, delete the object
    parameter_view_unchangeable.store(false);
    create_window_open = false;
    make_sensitive();
}

bool ParamViewUI::check_param_exists_callback(std::string name) {
    ParameterWithDescription param;
    return parameter_storage->get_parameter(name, param);
}

//Menu bar item handlers

void ParamViewUI::params_reload_handler() {
    parameter_storage->loadFile();
    read_storage_data();
}

void ParamViewUI::params_save_handler() {
    parameter_storage->storeFile();
}

void ParamViewUI::params_save_as_handler(std::string filename) {
    parameter_storage->storeFile(filename);
}

void ParamViewUI::params_load_file_handler(std::string filename) {
    parameter_storage->loadFile(filename);
    read_storage_data();    
}

void ParamViewUI::params_load_multiple_files_handler() {

}

void ParamViewUI::params_load_params_handler() {

}

void ParamViewUI::make_sensitive() {
    parent->set_sensitive(true);
}

void ParamViewUI::make_insensitive() {
    parent->set_sensitive(false);
}
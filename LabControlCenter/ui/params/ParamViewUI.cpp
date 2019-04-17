#include "ParamViewUI.hpp"

ParamViewUI::ParamViewUI() {
    params_builder = Gtk::Builder::create_from_file("ui/params/params.glade");

    params_builder->get_widget("parameters_box", parent);
    params_builder->get_widget("parameters_flow_top", parameters_flow_top);
    params_builder->get_widget("parameters_search", parameters_search);
    params_builder->get_widget("parameters_box_filter", parameters_box_filter);
    params_builder->get_widget("parameters_filter_description", parameters_filter_description);
    params_builder->get_widget("parameters_filter", parameters_filter);
    params_builder->get_widget("parameters_list_scroll_window", parameters_list_scroll_window);
    params_builder->get_widget("parameters_list_tree", parameters_list_tree);
    params_builder->get_widget("parameters_box_buttons", parameters_box_buttons);
    params_builder->get_widget("parameters_box_delete", parameters_box_delete);
    params_builder->get_widget("parameters_button_delete", parameters_button_delete);
    params_builder->get_widget("parameters_box_edit", parameters_box_edit);
    params_builder->get_widget("parameters_button_edit", parameters_button_edit);
    params_builder->get_widget("parameters_box_create", parameters_box_create);
    params_builder->get_widget("parameters_button_create", parameters_button_create);

    assert(parent);
    assert(parameters_flow_top);
    assert(parameters_search);
    assert(parameters_box_filter);
    assert(parameters_filter_description);
    assert(parameters_filter);
    assert(parameters_list_scroll_window);
    assert(parameters_list_tree);
    assert(parameters_box_buttons);
    assert(parameters_box_delete);
    assert(parameters_button_delete);
    assert(parameters_box_edit);
    assert(parameters_button_edit);
    assert(parameters_box_create);
    assert(parameters_button_create);

    //Create data model for parameters
    parameter_list_storage = Gtk::ListStore::create(model);
    parameters_list_tree->set_model(parameter_list_storage);

    //Create some sample rows for model
    Gtk::TreeModel::Row row = *(parameter_list_storage->append());
    row[model.column_name] = "some_bool";
    row[model.column_type] = "bool";
    row[model.column_value] = "true";
    row[model.column_info] = "does things";

    row = *(parameter_list_storage->append());
    row[model.column_name] = "an_int";
    row[model.column_type] = "int";
    row[model.column_value] = "1231234";
    row[model.column_info] = "does other things";

    row = *(parameter_list_storage->append());
    row[model.column_name] = "hey_a_list";
    row[model.column_type] = "int list";
    row[model.column_value] = "[3,2,6,8,2,1,5,6,222,444,555,777,111,12334554656253]";
    row[model.column_info] = "does listy things";

    //Use model, add it to the view
    parameters_list_tree->append_column("Name", model.column_name);
    parameters_list_tree->append_column("Type", model.column_type);
    parameters_list_tree->append_column("Value", model.column_value);
    parameters_list_tree->append_column("Info", model.column_info);

    //Set equal width for all columns
    for (int i = 0; i < 4; ++i) {
        parameters_list_tree->get_column(i)->set_resizable(true);
        parameters_list_tree->get_column(i)->set_min_width(20);
        parameters_list_tree->get_column(i)->set_fixed_width(50);
        parameters_list_tree->get_column(i)->set_expand(true);
    }
}

Gtk::Widget* ParamViewUI::get_parent() {
    return parent;
}
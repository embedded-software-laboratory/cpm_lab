#include "ParamViewUI.hpp"

ParamViewUI::ParamViewUI() {
    params_builder = Gtk::Builder::create_from_file("ui/params/params.glade");

    params_builder->get_widget("parameters_box", parent);
    params_builder->get_widget("parameters_box_top", parameters_box_top);
    params_builder->get_widget("parameters_flow_top", parameters_flow_top);
    params_builder->get_widget("parameters_search", parameters_search);
    params_builder->get_widget("parameters_box_filter", parameters_box_filter);
    params_builder->get_widget("parameters_filter_description", parameters_filter_description);
    params_builder->get_widget("parameters_filter", parameters_filter);
    params_builder->get_widget("parameters_box_top_names", parameters_box_top_names);
    params_builder->get_widget("parameters_list_scroll_window", parameters_list_scroll_window);
    params_builder->get_widget("parameters_list_viewport", parameters_list_viewport);
    params_builder->get_widget("parameters_list_tree", parameters_list_tree);
    params_builder->get_widget("parameters_box_buttons", parameters_box_buttons);
    params_builder->get_widget("parameters_box_delete", parameters_box_delete);
    params_builder->get_widget("parameters_button_delete", parameters_button_delete);
    params_builder->get_widget("parameters_box_edit", parameters_box_edit);
    params_builder->get_widget("parameters_button_edit", parameters_button_edit);
    params_builder->get_widget("parameters_box_create", parameters_box_create);
    params_builder->get_widget("parameters_button_create", parameters_button_create);

    assert(parent);
    assert(parameters_box_top);
    assert(parameters_flow_top);
    assert(parameters_search);
    assert(parameters_box_filter);
    assert(parameters_filter_description);
    assert(parameters_filter);
    assert(parameters_box_top_names);
    assert(parameters_list_scroll_window);
    assert(parameters_list_viewport);
    assert(parameters_list_tree);
    assert(parameters_box_buttons);
    assert(parameters_box_delete);
    assert(parameters_button_delete);
    assert(parameters_box_edit);
    assert(parameters_button_edit);
    assert(parameters_box_create);
    assert(parameters_button_create);
}

Gtk::Widget* ParamViewUI::get_parent() {
    return parent;
}
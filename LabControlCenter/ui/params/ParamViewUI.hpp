#pragma once

#include "defaults.hpp"
#include "ParamModelRecord.hpp"
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <gtkmm/liststore.h>
#include <cassert>
#include <string>

class ParamViewUI {
private:
    Glib::RefPtr<Gtk::Builder> params_builder;

    Gtk::Widget* parent;

    //Top box: search bar and filter
    Gtk::FlowBox* parameters_flow_top;
    Gtk::SearchEntry* parameters_search;
    Gtk::Box* parameters_box_filter;
    Gtk::Label* parameters_filter_description;
    Gtk::ComboBoxText* parameters_filter;

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
public:
    ParamViewUI();
    Gtk::Widget* get_parent();
    
    //Manipulate rows
    bool get_selected_row(std::string &name, std::string &type, std::string &value, std::string &info);
    //"Callback" function: Delete the row selected by the user if the delete button was clicked
    void delete_selected_row();
};
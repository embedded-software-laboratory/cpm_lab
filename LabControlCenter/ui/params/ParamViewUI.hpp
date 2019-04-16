#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

class ParamViewUI {
private:
    Gtk::Box* parameters_box;

    //Top box: search bar and filter
    Gtk::FlowBox* parameters_box_top;
    Gtk::FlowBoxChild* paramaters_box_top_left;
    Gtk::SearchEntry* parameters_search;
    Gtk::FlowBoxChild* paramaters_box_top_right;
    Gtk::Box* parameters_box_filter;
    Gtk::Label* parameters_filter_description;
    Gtk::ComboBoxText* parameters_filter;

    //Middle box: Parameter list
    Gtk::
public:
    ParamViewUI();
    Gtk::Box* get_parent();
}
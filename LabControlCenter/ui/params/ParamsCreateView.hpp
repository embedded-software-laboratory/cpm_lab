#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <cassert>
#include <string>
#include <functional>

class ParamsCreateView {
private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Window* window;
    Gtk::Widget* parent;
    Gtk::Box* params_create_box;
    Gtk::ButtonBox* params_create_buttons;
    Gtk::Button* params_create_abort_button;
    Gtk::Button* params_create_add_button;
    Gtk::Grid* params_create_values_grid;

    //Data to modify
    std::string name = "";
    std::string type = "";
    std::string value = "";
    std::string info = "";

    //Custom fields that allow to edit data
    Gtk::Entry* name_entry;
    Gtk::Entry* type_entry;
    Gtk::Entry* value_entry;
    Gtk::Entry* info_entry;

    //Function used by both constructors
    void init_members();

    //Callback function on close (must always be called!)
    std::function<void(std::string, std::string, std::string, std::string)> on_close_callback;

    //Callback functions for buttons
    void on_abort();
    void on_add();
public:
    ParamsCreateView(std::function<void(std::string, std::string, std::string, std::string)> on_close_callback);
    ParamsCreateView(std::function<void(std::string, std::string, std::string, std::string)> on_close_callback, std::string _name, std::string _type, std::string _value, std::string _info);
};
#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <cassert>

class ParamsCreateView {
private:
    Glib::RefPtr<Gtk::Builder> params_create_builder;

    Gtk::Widget* parent;
public:
    ParamsCreateView;
    Gtk::Widget* get_parent();
};
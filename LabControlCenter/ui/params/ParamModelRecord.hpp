#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

class ParamModelRecord : public Gtk::TreeModelColumnRecord {
public:
  ParamModelRecord() { 
      add(column_name);
      add(column_type);
      add(column_value);
      add(column_info);
    }

  Gtk::TreeModelColumn<Glib::ustring> column_name;
  Gtk::TreeModelColumn<Glib::ustring> column_type;
  Gtk::TreeModelColumn<Glib::ustring> column_value;
  Gtk::TreeModelColumn<Glib::ustring> column_info;
};
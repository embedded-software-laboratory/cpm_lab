#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

class LCCErrorModelRecord : public Gtk::TreeModelColumnRecord {
public:
  LCCErrorModelRecord() {
      add(timestamps); 
      add(error_content);
    }

  Gtk::TreeModelColumn<Glib::ustring> timestamps;
  Gtk::TreeModelColumn<Glib::ustring> error_content;
};
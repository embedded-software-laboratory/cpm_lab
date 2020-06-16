#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

class LoggerModelRecord : public Gtk::TreeModelColumnRecord {
public:
  LoggerModelRecord() { 
      add(log_id);
      add(log_content);
      add(log_stamp);
    }

  Gtk::TreeModelColumn<Glib::ustring> log_id;
  Gtk::TreeModelColumn<Glib::ustring> log_content;
  Gtk::TreeModelColumn<uint64_t> log_stamp;
};
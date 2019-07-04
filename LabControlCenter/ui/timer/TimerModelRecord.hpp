#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

class TimerModelRecord : public Gtk::TreeModelColumnRecord {
public:
  TimerModelRecord() { 
      add(column_id);
      add(column_last_message);
      add(column_participant_status);
      add(column_next_step);
    }

  Gtk::TreeModelColumn<Glib::ustring> column_id;
  Gtk::TreeModelColumn<Glib::ustring> column_last_message;
  Gtk::TreeModelColumn<Glib::ustring> column_participant_status;
  Gtk::TreeModelColumn<Glib::ustring> column_next_step;
};
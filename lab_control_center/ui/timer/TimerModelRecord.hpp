#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

/**
 * \class TimerModelRecord
 * \brief LCC UI GTK Tree...Record class that stores timer information: The timer user ID, last msg received from that timer, 
 * status of the user and next desired time step (simulated time)
 * \ingroup lcc_ui
 */
class TimerModelRecord : public Gtk::TreeModelColumnRecord {
public:
  /**
   * \brief Constructor
   * 
   * Sets the entries / columns for the TreeView that uses this record, here Timer ID | Time since last message | Participant status | Next time step
   */
  TimerModelRecord() { 
      add(column_id);
      add(column_last_message);
      add(column_participant_status);
      add(column_next_step);
    }

  //! Timer ID, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> column_id;
  //! Time since last message from this timer instance, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> column_last_message;
  //! Status of the timer / participant with the given ID, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> column_participant_status;
  //! Next time step of the timer (for simulated time), column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> column_next_step;
};
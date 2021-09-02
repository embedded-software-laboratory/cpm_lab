#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

/**
 * \class ProblemModelRecord
 * \brief A GTK Tree...Record to store a planning problem ID, goal speed and goal time
 * \ingroup lcc_ui
 */
class ProblemModelRecord : public Gtk::TreeModelColumnRecord {
public:
  /**
   * \brief Constructor
   * 
   * Sets the entries / columns for the TreeView that uses this record, here problem ID | goal speed | goal time.
   */
  ProblemModelRecord() {
      add(problem_id); 
      add(problem_goal_speed);
      add(problem_goal_time);
    }

  //! Commonroad planning problem ID, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> problem_id;
  //! Goal speed in a commonroad goal state, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> problem_goal_speed;
  //! Goal time in a commonroad goal state, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> problem_goal_time;
};
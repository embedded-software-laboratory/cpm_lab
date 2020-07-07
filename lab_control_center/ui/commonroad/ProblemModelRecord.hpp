#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

class ProblemModelRecord : public Gtk::TreeModelColumnRecord {
public:
  ProblemModelRecord() {
      add(problem_id); 
      add(problem_goal_speed);
      add(problem_goal_time);
    }

  Gtk::TreeModelColumn<Glib::ustring> problem_id;
  Gtk::TreeModelColumn<Glib::ustring> problem_goal_speed;
  Gtk::TreeModelColumn<Glib::ustring> problem_goal_time;
};
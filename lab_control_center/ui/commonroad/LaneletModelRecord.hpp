#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

/**
 * \class LaneletModelRecord
 * \brief A GTK Tree...Record to store lanelet info that cannot be drawn conveniently on the map
 * \ingroup lcc_ui
 */
class LaneletModelRecord : public Gtk::TreeModelColumnRecord {
public:
  /**
   * \brief Constructor
   * 
   * Sets the entries / columns for the TreeView that uses this record, here lanelet ID | lanelet type(s) | speed(2018) | user_one_way | user_bidirectional
   */
  LaneletModelRecord() {
      add(lanelet_id); 
      add(lanelet_type);
      add(user_one_way);
      add(user_bidirectional);
      add(speed_2018);
    }

  //! Commonroad lanelet ID, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> lanelet_id;
  //! Lanelet type, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> lanelet_type;
  //! Allowed lanelet users in driving direction, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> user_one_way;
  //! Allowed lanelet users bidirectional, column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> user_bidirectional;
  //! Lanelet speed limit (2018 specs), column of the TreeView that uses this record
  Gtk::TreeModelColumn<Glib::ustring> speed_2018;
};
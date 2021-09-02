#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

/**
 * \class LCCErrorModelRecord
 * \brief A GTK Tree...Record for storing error timestamps and content
 * \ingroup lcc_ui
 */
class LCCErrorModelRecord : public Gtk::TreeModelColumnRecord {
public:
  /**
   * \brief Constructor that defines the columns for a TreeView that uses this Record. Here: Timestamp | Error message
   */
  LCCErrorModelRecord() {
      add(timestamps); 
      add(error_content);
    }

  //! Timestamp column
  Gtk::TreeModelColumn<Glib::ustring> timestamps;
  //! Error message column
  Gtk::TreeModelColumn<Glib::ustring> error_content;
};
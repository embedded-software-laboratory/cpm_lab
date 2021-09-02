#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

/**
 * \class LoggerModelRecord
 * \brief A GTK Tree...Record for storing log ID, content and timestamp
 * \ingroup lcc_ui
 */
class LoggerModelRecord : public Gtk::TreeModelColumnRecord {
public:
  /**
   * \brief Constructor that defines the columns for the TreeView that uses this Record, in this case Log ID | Log content | Log timestamp
   */
  LoggerModelRecord() { 
      add(log_id);
      add(log_content);
      add(log_stamp);
    }

  //! Column for log IDs
  Gtk::TreeModelColumn<Glib::ustring> log_id;
  //! Column for log message contents
  Gtk::TreeModelColumn<Glib::ustring> log_content;
  //! Column for log timestamps
  Gtk::TreeModelColumn<uint64_t> log_stamp;
};
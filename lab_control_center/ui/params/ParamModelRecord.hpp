#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

/**
 * \class ParamModelRecord
 * \brief A LCC UI GTK Tree...Record for storing the name, type, value and additional information of a parameter
 * \ingroup lcc_ui
 */
class ParamModelRecord : public Gtk::TreeModelColumnRecord {
public:
  /**
   * \brief Constructor that defines the columns for a TreeView that uses this Record. Here: Param name | Param type | Param value | Additional info / description
   */
  ParamModelRecord() { 
      add(column_name);
      add(column_type);
      add(column_value);
      add(column_info);
    }

  //! Column for the parameter name
  Gtk::TreeModelColumn<Glib::ustring> column_name;
  //! Column for the parameter type
  Gtk::TreeModelColumn<Glib::ustring> column_type;
  //! Column for the parameter value
  Gtk::TreeModelColumn<Glib::ustring> column_value;
  //! Column for the parameter description
  Gtk::TreeModelColumn<Glib::ustring> column_info;
};
// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
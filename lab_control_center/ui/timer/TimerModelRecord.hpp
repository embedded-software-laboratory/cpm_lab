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
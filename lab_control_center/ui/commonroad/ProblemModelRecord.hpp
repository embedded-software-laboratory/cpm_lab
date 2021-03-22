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
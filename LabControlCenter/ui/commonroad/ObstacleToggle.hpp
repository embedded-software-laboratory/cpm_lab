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

/**
 * \brief This class contains all UI elements that control the desired setup state for one obstacle.
 * This allows for less redundant UI code.
 */

#include <cassert>
#include <gtkmm/builder.h>
#include <gtkmm.h>

#include <functional>
#include <iostream>
#include <sstream>

class ObstacleToggle 
{
public:
    ObstacleToggle(unsigned int _id);

    enum ToggleState{Simulated, On};

    //Getter
    Gtk::Widget* get_parent();

    //Setter
    void set_state(ToggleState state);
    void set_selection_callback(std::function<void(unsigned int, ToggleState)> _selection_callback); //If set, callback gets called on state change

private:
    void on_state_changed();
    ToggleState current_state;

    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::FlowBox* parent = nullptr;

    Gtk::Label* label = nullptr;

    Gtk::RadioButton* vehicle_sim = nullptr;
    Gtk::RadioButton* vehicle_on = nullptr;

    //Given values
    unsigned int id;
    std::function<void(unsigned int, ToggleState)> selection_callback;
};
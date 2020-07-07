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

#include "ObstacleToggle.hpp"

ObstacleToggle::ObstacleToggle(unsigned int _id) :
    id(_id % 256)
{
    builder = Gtk::Builder::create_from_file("ui/commonroad/obstacle_toggle.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("label", label);
    builder->get_widget("obstacle_switch", obstacle_switch);

    assert(parent);
    assert(label);
    assert(obstacle_switch);

    //Set label for vehicle toggle box
    std::stringstream label_str;
    label_str << "Obstacle " << id;
    label->set_text(label_str.str().c_str());

    current_state = ToggleState::Simulated;
    obstacle_switch->set_active(false);

    //Register callbacks
    obstacle_switch->property_active().signal_changed().connect(sigc::mem_fun(this, &ObstacleToggle::on_state_changed));
}

void ObstacleToggle::on_state_changed()
{
    if(obstacle_switch->get_active())
    {
        current_state = ToggleState::On;
    }
    else
    {
        current_state = ToggleState::Simulated;   
    }

    if(selection_callback) selection_callback(id, current_state);
}

Gtk::Widget* ObstacleToggle::get_parent()
{
    return parent;
}

void ObstacleToggle::set_state(ToggleState state)
{
    current_state = state;

    switch(state)
    {
        case ToggleState::On:
            obstacle_switch->set_active(true);
            break;

        case ToggleState::Simulated:
            obstacle_switch->set_active(false);
            break;
    }
}

void ObstacleToggle::set_selection_callback(std::function<void(unsigned int,ToggleState)> _selection_callback)
{
    selection_callback = _selection_callback;
}
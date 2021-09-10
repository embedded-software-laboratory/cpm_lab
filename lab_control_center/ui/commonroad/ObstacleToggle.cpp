#include "ObstacleToggle.hpp"

/**
 * \file ObstacleToggle.cpp
 * \ingroup lcc_ui
 */

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

ObstacleToggle::ToggleState ObstacleToggle::get_state()
{
    return current_state;
}

unsigned int ObstacleToggle::get_id()
{
    return id;
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
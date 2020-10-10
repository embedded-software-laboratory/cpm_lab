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

#include <gtkmm.h>
#include "defaults.hpp"
#include "TimeSeries.hpp"
#include <thread>
#include <mutex>
#include <sstream>
#include <vector>
#include "TrajectoryCommand.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "Visualization.hpp"
#include "Pose2D.hpp"
#include "cpm/get_time_ns.hpp"

#include "CommonroadObstacle.hpp"

#include "commonroad_classes/CommonRoadScenario.hpp"


using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;
using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, VehicleCommandTrajectory >;

class MapViewUi
{
    shared_ptr<TrajectoryCommand> trajectoryCommand;
    shared_ptr<CommonRoadScenario> commonroad_scenario;
    Gtk::DrawingArea* drawingArea;
    std::function<VehicleData()> get_vehicle_data;
    std::function<VehicleTrajectories()> get_vehicle_trajectory_command_callback;
    std::function<std::vector<Visualization>()> get_visualization_msgs_callback;
    Glib::Dispatcher update_dispatcher;
    std::thread draw_loop_thread;
    Cairo::RefPtr<Cairo::ImageSurface> image_car;
    Cairo::RefPtr<Cairo::ImageSurface> image_object;
    Cairo::RefPtr<Cairo::ImageSurface> image_map;
    VehicleData vehicle_data;

    //For visualization of commonroad data get from data storage object via callback
    std::function<std::vector<CommonroadObstacle>()> get_obstacle_data;

    // hold the path and related values temporarily, while the user draws with the mouse
    std::vector<Pose2D> path_painting_in_progress;
    int path_painting_in_progress_vehicle_id = -1;
    static constexpr double path_segment_delta_s = 0.01; // meter
    static constexpr double path_segment_max_delta_yaw = path_segment_delta_s * 2.6; // radian


    int vehicle_id_in_focus = -1;

    //(Leons comment, not his code) My understanding: These values can be set in MapViewUi.cpp and are applied whenever 
    //the callback function draw() is triggered (as I see it: after every mouse/scroll wheel event etc)
    //mouse_left_button is used to determine whether a vehicle trajectory should be drawn
    //mouse_right_button has now been added by me to allow for changing the view (by dragging) - this only works if the left button is not clicked as well!
    double zoom = 175;
    double pan_x = 100;
    double pan_y = 730;

    double mouse_x = 0;
    double mouse_y = 0;

    bool mouse_left_button = false;
    bool mouse_right_button = false; //The right mouse button or (new) the arrow keys can be used to move the view
    bool key_up = false;
    bool key_down = false;
    bool key_right = false;
    bool key_left = false;
    double key_move = 5.0; //Move value during key press, moved each time the dispatcher is called (change for higher / lower speed)

    //Used for dragging the view with the right mouse button
    double old_event_x = 0;
    double old_event_y = 0;



    void draw(const DrawingContext& ctx);

    void draw_grid(const DrawingContext& ctx);

    /**
     * \brief Draw the boundaries of the IPS / Lab to allow for fine-tuning the adjustment of commonroad maps, see where vehicle can be put etc.
     */
    void draw_lab_boundaries(const DrawingContext& ctx);

    void draw_vehicle_past_trajectory(
        const DrawingContext& ctx, 
        const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries
    );

    void draw_vehicle_body(
        const DrawingContext& ctx, 
        const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries, 
        uint8_t vehicle_id
    );
    /**
     * \brief Draw vehicles that were received in form of commonroad shape messages ('static' vehicles defined by the commonroad file)
     */
    void draw_vehicle_shape(const DrawingContext& ctx, CommonroadDDSShape& shape);
    std::pair<double, double> get_shape_center(CommonroadDDSShape& shape);
    void draw_path_painting(const DrawingContext& ctx);
    void draw_received_trajectory_commands(const DrawingContext& ctx);

    void draw_commonroad_obstacles(const DrawingContext& ctx);

    /**
     * /brief draw function that uses the viz callback to get all received viz commands and draws them on the screen
     */
    void draw_received_visualization_commands(const DrawingContext& ctx);

    bool is_valid_point_for_path(double x, double y);
    int find_vehicle_id_in_focus();

public:
    MapViewUi(
        shared_ptr<TrajectoryCommand> _trajectoryCommand,
        shared_ptr<CommonRoadScenario> _commonroad_scenario,
        std::function<VehicleData()> get_vehicle_data_callback,
        std::function<VehicleTrajectories()> _get_vehicle_trajectory_command_callback,
        std::function<std::vector<CommonroadObstacle>()> _get_obstacle_data,
        std::function<std::vector<Visualization>()> _get_visualization_msgs_callback
    );
    Gtk::DrawingArea* get_parent();
};
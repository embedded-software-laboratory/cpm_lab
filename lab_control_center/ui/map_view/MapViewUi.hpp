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
#include "VehicleCommandPathTracking.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "Visualization.hpp"
#include "Pose2D.hpp"
#include "cpm/get_time_ns.hpp"

#include "CommonroadObstacle.hpp"

#include "commonroad_classes/CommonRoadScenario.hpp"
#include "LCCErrorLogger.hpp"

/**
 * \brief Used in a lot of classes, provides a reference to the drawing context of the map view, 
 * in which e.g. the map, commonroad objects and visualization messages are drawn
 * \ingroup lcc_ui
 */
using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;
/**
 * \brief Maps vehicle ID to map: Data ID, TimeSeries data (e.g. battery->battery data)
 * \ingroup lcc_ui
 */
using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
/**
 * \brief Maps vehicle ID to current vehicle trajectory
 * \ingroup lcc_ui
 */
using VehicleTrajectories = map<uint8_t, VehicleCommandTrajectory >;
/**
 * \brief Maps vehicle ID to current vehicle path tracking
 * \ingroup lcc_ui
 */
using VehiclePathTracking = map<uint8_t, VehicleCommandPathTracking >;

/**
 * \class MapViewUi
 * \brief UI class for showing the map view of the LCC, including vehicles, map, trajectories and visualization commands
 * \ingroup lcc_ui
 */
class MapViewUi
{
    //! Reference to TrajectoryCommand class object, which is used to create trajectories for vehicles from trajectories drawn by the user in the map view
    shared_ptr<TrajectoryCommand> trajectoryCommand;
    //! Reference to the current commonroad scenario object, which is used for drawing objects and to find out which additional information to draw
    shared_ptr<CommonRoadScenario> commonroad_scenario;
    //! GTK area to draw on / to show the map, obstacles etc, also parent of the Map View UI element
    Gtk::DrawingArea* drawingArea;
    //! Callback to get vehicle data (position etc.) for drawing
    std::function<VehicleData()> get_vehicle_data;
    //! Callback to get vehicle trajectories for drawing
    std::function<VehicleTrajectories()> get_vehicle_trajectory_command_callback;
    //! Callback to get vehicle path tracking to draw trajectory interpolations
    std::function<VehiclePathTracking()> get_vehicle_path_tracking_command_callback;
    //! Callback to get received visualization messages, which are drawn on the map view as well (lines, circles, text etc.)
    std::function<std::vector<Visualization>()> get_visualization_msgs_callback;
    //! GTK dispatcher to connect to GTK's UI thread, for all drawing operations
    Glib::Dispatcher update_dispatcher;
    //! Calls update_dispatcher every 20ms for smooth map updates
    std::thread draw_loop_thread;
    //! Image object for the car
    Cairo::RefPtr<Cairo::ImageSurface> image_car;
    //! Image object for an object, currently not in use
    Cairo::RefPtr<Cairo::ImageSurface> image_object;
    //! Image object for the default map, not in use when commonroad maps are loaded
    Cairo::RefPtr<Cairo::ImageSurface> image_map;
    //! Image object for the labcam, to show the cam in the map view
    Cairo::RefPtr<Cairo::ImageSurface> image_labcam;
    //! Storage containing current vehicle data obtained with get_vehicle_data
    VehicleData vehicle_data;

    //! For visualization of / drawing commonroad data, get obstacle information from data storage object via callback
    std::function<std::vector<CommonroadObstacle>()> get_obstacle_data;

    //! hold the path and related values temporarily, while the user draws with the mouse
    std::vector<Pose2D> path_painting_in_progress;
    //! Vehicle ID / car for which the path is painted
    int path_painting_in_progress_vehicle_id = -1;
    //! meter, used in pose computation
    static constexpr double path_segment_delta_s = 0.01; 
    //! radian, used in pose computation
    static constexpr double path_segment_max_delta_yaw = path_segment_delta_s * 2.6; // radian

    //! Vehicle that is currently in focus, mostly used in context of path painting / starting to paint a path and to draw a disk around the current vehicle in focus
    int vehicle_id_in_focus = -1;

    //(Leons comment, not his code) My understanding: These values can be set in MapViewUi.cpp and are applied whenever 
    //the callback function draw() is triggered (as I see it: after every mouse/scroll wheel event etc)
    //mouse_left_button is used to determine whether a vehicle trajectory should be drawn
    //mouse_right_button has now been added by me to allow for changing the view (by dragging) - this only works if the left button is not clicked as well!
    //! For scaling the view, values between 30 and 900 are allowed (not to be confused with commonroad map transformations)
    double zoom = 175;
    //! For translating the view in x direction, can e.g. be done with the keyboard (not to be confused with commonroad map transformations)
    double pan_x = 100;
    //! For translating the view in y direction, can e.g. be done with the keyboard (not to be confused with commonroad map transformations)
    double pan_y = 730; 
    //! For rotating the view; rotation is done around the origin (rotation_fixpoint_x and rotation_fixpoint_y) (not to be confused with commonroad map transformations)
    double rotation = 0; //[rad]

    //! point, which doesn't change when rotating (corresponds to map center) -> origin to rotate around
    const double rotation_fixpoint_x = 2.25;
    //! point, which doesn't change when rotating (corresponds to map center) -> origin to rotate around
    const double rotation_fixpoint_y = 2;

    //! Mouse x position in "world" coordinate (not canvas coordinate, which might be translated / scaled / ...)
    double mouse_x = 0;
    //! Mouse y position in "world" coordinate (not canvas coordinate, which might be translated / scaled / ...)
    double mouse_y = 0;

    //! Tells if the left mouse button is currently pressed. The left mouse button can be used e.g. to draw a trajectory for a selected vehicle
    bool mouse_left_button = false;
    //! Tells if the right mouse button is currently pressed. The right mouse button or (new) the arrow keys can be used to move the view
    bool mouse_right_button = false; 
    //! If the up key is currently pressed, for translating the view
    bool key_up = false;
    //! If the down key is currently pressed, for translating the view
    bool key_down = false;
    //! If the right key is currently pressed, for translating the view
    bool key_right = false;
    //! If the left key is currently pressed, for translating the view
    bool key_left = false;
    //! Move value during key press, moved each time the dispatcher is called (change for higher / lower speed)
    double key_move = 5.0; 

    //! Used for dragging the view with the right mouse button by getting the event / mouse movement distance
    double old_event_x = 0;
    //! Used for dragging the view with the right mouse button by getting the event / mouse movement distance
    double old_event_y = 0;


    /**
     * \brief "Master" draw function, invoked regularly on the map view / drawingArea, calls all other draw functions
     * \param ctx The drawing context, to draw on the map view
     */
    void draw(const DrawingContext& ctx);

    /**
     * \brief Deprecated, draw a grid in the map view
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_grid(const DrawingContext& ctx);
    /**
     * \brief Draw labcam image at the correct position
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_labcam(const DrawingContext& ctx);

    /**
     * \brief Draw the boundaries of the IPS / Lab to allow for fine-tuning the adjustment of commonroad maps, see where vehicle can be put etc.
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_lab_boundaries(const DrawingContext& ctx);

    /**
     * \brief Draws the past trajectory of the vehicle
     * \param ctx The drawing context, to draw on the map view
     * \param vehicle_timeseries Gives past vehicle pose_x and pose_y values for drawing the past trajectory between them
     */
    void draw_vehicle_past_trajectory(
        const DrawingContext& ctx, 
        const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries
    );

    /**
     * \brief Draws car image with vehicle ID on top, to show the current position of the vehicle
     * \param ctx The drawing context, to draw on the map view
     * \param vehicle_timeseries Gives current vehicle position and orientation
     * \param vehicle_id ID to draw on top of the car image, to visually identify the drawn vehicle
     */
    void draw_vehicle_body(
        const DrawingContext& ctx, 
        const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries, 
        uint8_t vehicle_id
    );

    /**
     * \brief Draw vehicles that were received in form of commonroad shape messages ('static' vehicles defined by the commonroad file)
     * \param ctx The drawing context, to draw on the map view
     * \param shape Commonroad shape to draw, e.g. a rectangle
     */
    void draw_vehicle_shape(const DrawingContext& ctx, CommonroadDDSShape& shape);

    /**
     * \brief Get the center (x, y) of a commonroad shape, i.e. the mean value of its corner points etc.
     * WARNING: Does not check for potential overflow during computation
     * \param shape The shape to get the center of
     */
    std::pair<double, double> get_shape_center(CommonroadDDSShape& shape);

    /**
     * \brief Draw path drawn by the user with the mouse as desired vehicle trajectory
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_path_painting(const DrawingContext& ctx);

    /**
     * \brief Draw the vehicle's future trajectory, with one color for past and another for future parts of the trajectory
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_received_trajectory_commands(const DrawingContext& ctx);

    /**
     * \brief Draw received path tracking
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_received_path_tracking_commands(const DrawingContext& ctx);

    /**
     * \brief Draw all received commonroad obstacles from get_obstacle_data
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_commonroad_obstacles(const DrawingContext& ctx);

    /**
     * \brief draw function that uses the viz callback to get all received viz commands and draws them on the screen
     * \param ctx The drawing context, to draw on the map view
     */
    void draw_received_visualization_commands(const DrawingContext& ctx);

    /**
     * \brief TODO! Deprecated? Never used or defined.
     * \param x x coordinate
     * \param y y coordinate
     */
    bool is_valid_point_for_path(double x, double y);

    /**
     * \brief Determine vehicle ID in focus, by taking a look at the current mouse position and vehicle time series.
     * Is in focus if the mouse hovers over one of the vehicles shown on the map.
     */
    int find_vehicle_id_in_focus();

public:
    /**
     * \brief Constructor of the map view, which shows the current map, vehicles, commonroad objects, visualizations etc.
     * \param _trajectoryCommand Reference to TrajectoryCommand class object, to create trajectories for vehicles from trajectories drawn by the user
     * \param _commonroad_scenario Reference to commonroad scenario object, used for drawing objects and additional information
     * \param get_vehicle_data_callback Callback to get vehicle data (position etc.) for drawing
     * \param _get_vehicle_trajectory_command_callback Callback to get vehicle trajectories for drawing
     * \param _get_vehicle_path_tracking_command_callback Callback to get vehicle path tracking for drawing
     * \param _get_obstacle_data For visualization of / drawing commonroad data, get obstacle information from data storage object via callback
     * \param _get_visualization_msgs_callback Callback to get received visualization messages, which are drawn on the map view as well (lines, circles, text etc.)
     */
    MapViewUi(
        shared_ptr<TrajectoryCommand> _trajectoryCommand,
        shared_ptr<CommonRoadScenario> _commonroad_scenario,
        std::function<VehicleData()> get_vehicle_data_callback,
        std::function<VehicleTrajectories()> _get_vehicle_trajectory_command_callback,
        std::function<VehiclePathTracking()> _get_vehicle_path_tracking_command_callback,
        std::function<std::vector<CommonroadObstacle>()> _get_obstacle_data,
        std::function<std::vector<Visualization>()> _get_visualization_msgs_callback
    );

    /**
     * \brief Function to get the parent widget, so that this UI element can be placed within another UI element
     */
    Gtk::DrawingArea* get_parent();

    /**
     * \brief rotates the map view by rotation [deg] counterclockwise
     * \param rotation amount to rotate the map view by
     */
    void rotate_by(double rotation);
};
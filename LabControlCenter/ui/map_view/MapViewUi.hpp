#pragma once

#include <gtkmm.h>
#include "defaults.hpp"
#include "TimeSeries.hpp"
#include "Point.hpp"
#include <thread>
#include <vector>
#include "TrajectoryCommand.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "Visualization.hpp"
#include "Pose2D.hpp"


using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;
using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;
using VehicleTrajectories = map<uint8_t, map<uint64_t, TrajectoryPoint> >;

class MapViewUi
{
    shared_ptr<TrajectoryCommand> trajectoryCommand;
    Gtk::DrawingArea* drawingArea;
    std::function<VehicleData()> get_vehicle_data;
    std::function<VehicleTrajectories()> get_vehicle_trajectory_command_callback;
    std::function<std::vector<Visualization>()> get_visualization_msgs_callback;
    Glib::Dispatcher update_dispatcher;
    std::thread draw_loop_thread;
    Cairo::RefPtr<Cairo::ImageSurface> image_car;
    VehicleData vehicle_data;


    // hold the path and related values temporarily, while the user draws with the mouse
    std::vector<Pose2D> path_painting_in_progress;
    int path_painting_in_progress_vehicle_id = -1;
    static constexpr double path_segment_delta_s = 0.01; // meter
    static constexpr double path_segment_max_delta_yaw = path_segment_delta_s * 2.6; // radian


    int vehicle_id_in_focus = -1;

    double zoom = 175;
    double pan_x = 100;
    double pan_y = 730;

    double mouse_x = 0;
    double mouse_y = 0;

    bool mouse_left_button = false;



    void draw(const DrawingContext& ctx);

    void draw_grid(const DrawingContext& ctx);

    void draw_vehicle_past_trajectory(
        const DrawingContext& ctx, 
        const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries
    );

    void draw_vehicle_body(
        const DrawingContext& ctx, 
        const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries, 
        uint8_t vehicle_id
    );
    void draw_path_painting(const DrawingContext& ctx);
    void draw_received_trajectory_commands(const DrawingContext& ctx);

    /**
     * /brief draw function that uses the viz callback to get all received viz commands and draws them on the screen
     */
    void draw_received_visualization_commands(const DrawingContext& ctx);

    bool is_valid_point_for_path(double x, double y);
    int find_vehicle_id_in_focus();

public:
    MapViewUi(
        shared_ptr<TrajectoryCommand> _trajectoryCommand,
        std::function<VehicleData()> get_vehicle_data_callback,
        std::function<VehicleTrajectories()> _get_vehicle_trajectory_command_callback,
        std::function<std::vector<Visualization>()> _get_visualization_msgs_callback
    );
    Gtk::DrawingArea* get_parent();
};
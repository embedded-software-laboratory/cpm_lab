#pragma once

#include <gtkmm.h>
#include "defaults.hpp"
#include "TimeSeries.hpp"
#include <thread>

struct Point
{
    double x = 0;
    double y = 0;
    Point(double x,double y):x(x),y(y){}
};


using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;
using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

class MapViewUi
{
    Gtk::DrawingArea* drawingArea;
    std::function<VehicleData()> get_vehicle_data;
    Glib::Dispatcher update_dispatcher;
    std::thread draw_loop_thread;
    Cairo::RefPtr<Cairo::ImageSurface> image_car;
    VehicleData vehicle_data;


    // holds the path and related values temporarily, while the user draws with the mouse
    std::vector<Point> path_painting_in_progress;
    int path_painting_in_progress_vehicle_id = -1;
    double path_painting_in_progress_yaw = 0; // radian
    const double path_segment_length = 0.3; // meter
    const double path_segment_max_angle = 0.7; // radian


    int vehicle_in_focus = -1;

    double zoom = 200.5;
    double pan_x = 318.2;
    double pan_y = 819;

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


    int find_vehicle_id_in_focus();

public:
    explicit MapViewUi(std::function<VehicleData()> get_vehicle_data_callback);
    Gtk::DrawingArea* get_parent();
};
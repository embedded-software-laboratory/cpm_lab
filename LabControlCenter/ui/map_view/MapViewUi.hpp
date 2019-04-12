#pragma once

#include <gtkmm.h>
#include "defaults.hpp"
#include "TimeSeries.hpp"
#include <thread>


using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;
using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

class MapViewUi
{
    Gtk::DrawingArea* drawingArea;
    std::function<VehicleData()> get_vehicle_data;
    Glib::Dispatcher update_dispatcher;
    std::thread draw_loop_thread;
    Cairo::RefPtr<Cairo::ImageSurface> image_car;
    double zoom = 200.5;
    double pan_x = 318.2;
    double pan_y = 819;

    double mouse_x = 0;
    double mouse_y = 0;

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

public:
    explicit MapViewUi(std::function<VehicleData()> get_vehicle_data_callback);
    Gtk::DrawingArea* get_parent();
};
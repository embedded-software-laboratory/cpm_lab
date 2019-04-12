#pragma once

#include <gtkmm.h>
#include "defaults.hpp"
#include "TimeSeries.hpp"
#include "cpm/Timer.hpp"

using VehicleData = map<uint8_t, map<string, shared_ptr<TimeSeries> > >;

class MapViewUi
{
    Gtk::DrawingArea* drawingArea;
    std::function<VehicleData()> get_vehicle_data;
    Glib::Dispatcher update_dispatcher;
    shared_ptr<cpm::Timer> update_loop;
    Cairo::RefPtr<Cairo::ImageSurface> image_car;
    double zoom = 200.5;
    double pan_x = 318.2;
    double pan_y = 819;

public:
    explicit MapViewUi(std::function<VehicleData()> get_vehicle_data_callback);
    Gtk::DrawingArea* get_parent();
};
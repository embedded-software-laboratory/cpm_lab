#pragma once

#include <gtkmm.h>
#include "defaults.hpp"
#include "TimeSeries.hpp"
#include "AbsoluteTimer.hpp"

class MapViewUi
{
    shared_ptr<Gtk::Window> window;
    Gtk::DrawingArea* drawingArea;
    const map<uint8_t, map<string, shared_ptr<TimeSeries> > >& vehicle_data;
    Glib::Dispatcher update_dispatcher;
    shared_ptr<AbsoluteTimer> update_loop;
    Cairo::RefPtr<Cairo::ImageSurface> image_car;

public:
    explicit MapViewUi(const map<uint8_t, map<string, shared_ptr<TimeSeries> > >&);
    Gtk::Window& get_window();    
};
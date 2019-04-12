#include "MapViewUi.hpp"
#include <cassert>
#include <glibmm/main.h>

MapViewUi::MapViewUi(std::function<VehicleData()> get_vehicle_data_callback)
{
    this->get_vehicle_data = get_vehicle_data_callback;

    drawingArea = Gtk::manage(new Gtk::DrawingArea());
    drawingArea->set_double_buffered();

    image_car = Cairo::ImageSurface::create_from_png("ui/map_view/car_small.png");
    
    drawingArea->show();


    update_loop = cpm::Timer::create("LabControlCenterMap",40000000ull, 0, false, false);
    update_loop->start_async([&](uint64_t t_now){ update_dispatcher.emit(); });

    update_dispatcher.connect([&](){ drawingArea->queue_draw(); });

    drawingArea->add_events(Gdk::SCROLL_MASK);


    drawingArea->signal_scroll_event().connect([&](GdkEventScroll* event){

        double zoom_speed = 1;

        if(event->direction == GDK_SCROLL_DOWN && zoom > 30) 
        {
            zoom_speed = 1.0/1.1;
        }

        if(event->direction == GDK_SCROLL_UP && zoom < 900)
        {
            zoom_speed = 1.1;            
        } 

        if(zoom_speed != 1)
        {
            pan_x = event->x - zoom_speed * (event->x - pan_x);
            pan_y = event->y - zoom_speed * (event->y - pan_y);
            zoom *= zoom_speed;
        }

        std::cout << pan_x << "  " << pan_y << "  " << zoom << std::endl;

        return true; 
    });


    drawingArea->signal_draw().connect([&](const ::Cairo::RefPtr< ::Cairo::Context >& ctx)->bool {

        ctx->save();
        {
            ctx->translate(pan_x, pan_y);
            ctx->scale(zoom, zoom);


            // Draw grid
            ctx->save();
            {
                ctx->scale(.1, -.1);
                for (int i = 0; i <= 45; ++i)
                {
                    if(i == 0) {
                        ctx->set_line_width(0.15);
                    }
                    else if(i % 10 == 0) {
                        ctx->set_line_width(0.05);
                    }
                    else {
                        ctx->set_line_width(0.01);
                    }

                    ctx->move_to(i,0);
                    ctx->line_to(i,40);
                    ctx->stroke();

                    if(i <= 40)
                    {
                        ctx->move_to(0,i);
                        ctx->line_to(45,i);
                        ctx->stroke();
                    }
                }    
            }
            ctx->restore();


            auto vehicle_data = this->get_vehicle_data();
            for(const auto& entry : vehicle_data) {
                const auto vehicle_id = entry.first;
                const auto& vehicle_sensor_timeseries = entry.second;

                if(vehicle_sensor_timeseries.at("pose_x")->has_new_data(1.0))
                {

                    // Draw vehicle trajectory
                    {
                        vector<double> trajectory_x = vehicle_sensor_timeseries.at("pose_x")->get_last_n_values(100);
                        vector<double> trajectory_y = vehicle_sensor_timeseries.at("pose_y")->get_last_n_values(100);
                        for (size_t i = 1; i < trajectory_x.size(); ++i)
                        {
                            if(i == 1) ctx->move_to(trajectory_x[i], -trajectory_y[i]);
                            else ctx->line_to(trajectory_x[i], -trajectory_y[i]);
                        }
                        ctx->set_source_rgb(1,0,0);
                        ctx->set_line_width(0.01);
                        ctx->stroke();
                    }

                    // Draw vehicle
                    ctx->save();
                    {                        
                        const double x = vehicle_sensor_timeseries.at("pose_x")->get_latest_value();
                        const double y = vehicle_sensor_timeseries.at("pose_y")->get_latest_value();
                        const double yaw = vehicle_sensor_timeseries.at("pose_yaw")->get_latest_value();

                        ctx->translate(x,-y);
                        ctx->rotate(-yaw);

                        const double LF = 0.196;
                        const double LR = 0.028;
                        //const double WH = 0.054;

                        ctx->save();
                        {
                            const double scale = 0.224/image_car->get_width();
                            ctx->translate( (LF+LR)/2-LR ,0);
                            ctx->scale(scale, scale);
                            ctx->translate(-image_car->get_width()/2, -image_car->get_height()/2);
                            ctx->set_source(image_car,0,0);
                            ctx->paint();
                        }
                        ctx->restore();


                        ctx->save();
                        {
                            ctx->translate(0.04, 0);
                            const double scale = 0.01;
                            ctx->rotate(yaw);
                            ctx->scale(scale, scale);
                            ctx->move_to(0,0);
                            Cairo::TextExtents extents;
                            ctx->get_text_extents(to_string(vehicle_id), extents);

                            ctx->move_to(-extents.width/2 - extents.x_bearing, -extents.height/2 - extents.y_bearing);
                            ctx->set_source_rgb(1,1,1);
                            ctx->show_text(to_string(vehicle_id));

                            ctx->move_to(-extents.width/2 - extents.x_bearing - 0.6, -extents.height/2 - extents.y_bearing - 0.4);
                            ctx->set_source_rgb(1,.1,.1);
                            ctx->show_text(to_string(vehicle_id));
                        }
                        ctx->restore();

                        /*ctx->move_to(-LR, WH);
                        ctx->line_to(LF, WH);
                        ctx->line_to(LF, -WH);
                        ctx->line_to(-LR, -WH);
                        ctx->line_to(-LR, WH);
                        ctx->stroke();*/
                    }
                    ctx->restore();
                }
            }
        }
        ctx->restore();

        return true;
    });
}

Gtk::DrawingArea* MapViewUi::get_parent()
{
    return drawingArea;
}
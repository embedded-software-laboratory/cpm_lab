#include "MapViewUi.hpp"
#include <cassert>

MapViewUi::MapViewUi(const map<uint8_t, map<string, shared_ptr<TimeSeries> > >& _vehicle_data) 
:vehicle_data(_vehicle_data)
{

    window = make_shared<Gtk::Window>();
    drawingArea = Gtk::manage(new Gtk::DrawingArea());
    drawingArea->set_double_buffered();

    image_car = Cairo::ImageSurface::create_from_png("ui/map_view/car_small.png");
    
    window->add(*drawingArea);


    update_loop = make_shared<AbsoluteTimer>(0, 40000000, 0, 0, [&](){ update_dispatcher.emit(); });
    update_dispatcher.connect([&](){ drawingArea->queue_draw(); });


    window->set_title("Map View");
    window->maximize();
    window->show_all();

    drawingArea->signal_draw().connect([&](const ::Cairo::RefPtr< ::Cairo::Context >& ctx)->bool {
        ctx->save();
        {
            ctx->translate(drawingArea->get_allocated_width()/2, drawingArea->get_allocated_height()/2);
            ctx->scale(300, 300);

            ctx->set_line_width(0.005);

            const int grid_size = 10;
            for (int i = -grid_size; i <= grid_size; ++i)
            {
                ctx->move_to(i,-grid_size);
                ctx->line_to(i,grid_size);
                ctx->stroke();

                ctx->move_to(-grid_size,i);
                ctx->line_to(grid_size,i);
                ctx->stroke();
            }

            for(const auto& entry : vehicle_data) {
                const auto vehicle_id = entry.first;
                const auto& vehicle_sensor_timeseries = entry.second;

                if(vehicle_sensor_timeseries.at("pose_x")->has_new_data(1.0))
                {
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
                            ctx->translate((LF+LR)/2-LR,0);
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

    window->signal_delete_event().connect([&](GdkEventAny*)->bool{
        exit(0);
        return false;
    });
}


Gtk::Window& MapViewUi::get_window()
{
    return *window;
}

#include "MapViewUi.hpp"
#include <cassert>
#include <glibmm/main.h>


MapViewUi::MapViewUi(std::function<VehicleData()> get_vehicle_data_callback)
{
    this->get_vehicle_data = get_vehicle_data_callback;

    drawingArea = Gtk::manage(new Gtk::DrawingArea());
    drawingArea->set_double_buffered();
    drawingArea->show();

    image_car = Cairo::ImageSurface::create_from_png("ui/map_view/car_small.png");
    
    update_dispatcher.connect([&](){ 
        vehicle_data = this->get_vehicle_data();
        drawingArea->queue_draw(); 
    });

    draw_loop_thread = std::thread([&](){
        while(1) {
            usleep(40000);
            update_dispatcher.emit();
        }
    });

    drawingArea->add_events(Gdk::SCROLL_MASK);
    drawingArea->add_events(Gdk::BUTTON_PRESS_MASK);
    drawingArea->add_events(Gdk::BUTTON_RELEASE_MASK);
    drawingArea->add_events(Gdk::POINTER_MOTION_MASK);


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

        return true; 
    });

    drawingArea->signal_button_press_event().connect([&](GdkEventButton* event) {
        if(event->button == 1) mouse_left_button = true;

        // start path drawing mode
        if(mouse_left_button)
        {
            path_painting_in_progress_vehicle_id = find_vehicle_id_in_focus();
            if(path_painting_in_progress_vehicle_id >= 0)
            {
                path_painting_in_progress.clear();
                auto vehicle_timeseries = vehicle_data.at(path_painting_in_progress_vehicle_id);

                path_painting_in_progress_yaw = vehicle_timeseries.at("pose_yaw")->get_latest_value();
                Point start_point(
                    vehicle_timeseries.at("pose_x")->get_latest_value(),
                    vehicle_timeseries.at("pose_y")->get_latest_value()
                );
                path_painting_in_progress.push_back(start_point);
            }
        }

        return true;
    });

    drawingArea->signal_button_release_event().connect([&](GdkEventButton* event) {
        if(event->button == 1) mouse_left_button = false;
        return true;
    });

    drawingArea->signal_motion_notify_event().connect([&](GdkEventMotion* event) {
        mouse_x = (event->x - pan_x) / zoom;
        mouse_y = -(event->y - pan_y) / zoom;

        vehicle_in_focus = find_vehicle_id_in_focus();
        return true;
    });


    drawingArea->signal_draw().connect([&](const DrawingContext& ctx)->bool {
        this->draw(ctx); 
        return true;
    });
}

int MapViewUi::find_vehicle_id_in_focus()
{
    int id = -1;
    double dist_sq_min = 1e300;

    for(const auto& entry : vehicle_data) {
        const auto vehicle_id = entry.first;
        const auto& vehicle_timeseries = entry.second;

        if(!vehicle_timeseries.at("pose_x")->has_new_data(1.0)) continue;

        double dx = mouse_x - vehicle_timeseries.at("pose_x")->get_latest_value();
        double dy = mouse_y - vehicle_timeseries.at("pose_y")->get_latest_value();

        double dist_sq = dx*dx + dy*dy;
        if(dist_sq < 0.04 && dist_sq < dist_sq_min)
        {
            id = vehicle_id;
            dist_sq_min = dist_sq;
        }
    }
    return id;
}


void MapViewUi::draw(const DrawingContext& ctx)
{
    ctx->save();
    {
        ctx->translate(pan_x, pan_y);
        ctx->scale(zoom, -zoom);

        draw_grid(ctx);

        // TODO! clean this up
        if(mouse_left_button)
        {
            ctx->set_source_rgb(0,.6,0);
            ctx->arc(mouse_x, mouse_y, 0.1, 0.0, 2 * M_PI);
            ctx->fill();
        }

        if(vehicle_in_focus >= 0)
        {
            ctx->set_source_rgba(0,1,1,0.5);
            ctx->arc(
                vehicle_data.at(vehicle_in_focus).at("pose_x")->get_latest_value(),
                vehicle_data.at(vehicle_in_focus).at("pose_y")->get_latest_value(),
                0.2, 0.0, 2 * M_PI
            );
            ctx->fill();
        }

        if(!path_painting_in_progress.empty() && path_painting_in_progress_vehicle_id >= 0)
        {
            ctx->set_source_rgba(0,0,1,0.5);
            ctx->arc(
                path_painting_in_progress.back().x,
                path_painting_in_progress.back().y,
                path_segment_length, 
                path_painting_in_progress_yaw - path_segment_max_angle, 
                path_painting_in_progress_yaw + path_segment_max_angle
            );
            ctx->line_to(
                path_painting_in_progress.back().x,
                path_painting_in_progress.back().y
            );
            ctx->fill();

        }




        for(const auto& entry : vehicle_data) {
            const auto vehicle_id = entry.first;
            const auto& vehicle_timeseries = entry.second;

            if(vehicle_timeseries.at("pose_x")->has_new_data(1.0))
            {
                draw_vehicle_past_trajectory(ctx, vehicle_timeseries);
                draw_vehicle_body(ctx, vehicle_timeseries, vehicle_id);
            }
        }
    }
    ctx->restore();
}

void MapViewUi::draw_grid(const DrawingContext& ctx)
{
    ctx->save();
    {
        ctx->scale(.1, .1);
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
}

void MapViewUi::draw_vehicle_past_trajectory(const DrawingContext& ctx, const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries)
{
    vector<double> trajectory_x = vehicle_timeseries.at("pose_x")->get_last_n_values(100);
    vector<double> trajectory_y = vehicle_timeseries.at("pose_y")->get_last_n_values(100);
    for (size_t i = 1; i < trajectory_x.size(); ++i)
    {
        if(i == 1) ctx->move_to(trajectory_x[i], trajectory_y[i]);
        else ctx->line_to(trajectory_x[i], trajectory_y[i]);
    }
    ctx->set_source_rgb(1,0,0);
    ctx->set_line_width(0.01);
    ctx->stroke();
}

void MapViewUi::draw_vehicle_body(const DrawingContext& ctx, const map<string, shared_ptr<TimeSeries>>& vehicle_timeseries, uint8_t vehicle_id)
{
    ctx->save();
    {                        
        const double x = vehicle_timeseries.at("pose_x")->get_latest_value();
        const double y = vehicle_timeseries.at("pose_y")->get_latest_value();
        const double yaw = vehicle_timeseries.at("pose_yaw")->get_latest_value();

        ctx->translate(x,y);
        ctx->rotate(yaw);

        const double LF = 0.196;
        const double LR = 0.028;
        const double WH = 0.054;

        // Draw car image
        ctx->save();
        {
            // TODO vehicle coordinate system definition changed, fix here accordingly
            const double scale = 0.224/image_car->get_width();
            ctx->translate( (LF+LR)/2-LR ,0);
            ctx->scale(scale, scale);
            ctx->translate(-image_car->get_width()/2, -image_car->get_height()/2);
            ctx->set_source(image_car,0,0);
            ctx->paint();
        }
        ctx->restore();

        // Draw vehicle ID
        ctx->save();
        {
            ctx->translate(0.04, 0);
            const double scale = 0.01;
            ctx->rotate(-yaw);
            ctx->scale(scale, -scale);
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

        // Draw vehicle box
        /*ctx->move_to(-LR, WH);
        ctx->line_to(LF, WH);
        ctx->line_to(LF, -WH);
        ctx->line_to(-LR, -WH);
        ctx->line_to(-LR, WH);
        ctx->stroke();*/
    }
    ctx->restore();
}




Gtk::DrawingArea* MapViewUi::get_parent()
{
    return drawingArea;
}
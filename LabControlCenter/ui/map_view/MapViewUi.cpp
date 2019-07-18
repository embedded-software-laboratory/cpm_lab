#include "MapViewUi.hpp"
#include <cassert>
#include <glibmm/main.h>

#include "../vehicle_raspberry_firmware/src/TrajectoryInterpolation.hpp"
#include "../vehicle_raspberry_firmware/src/TrajectoryInterpolation.cxx"


MapViewUi::MapViewUi(
    shared_ptr<TrajectoryCommand> _trajectoryCommand,
    std::function<VehicleData()> get_vehicle_data_callback,
    std::function<VehicleTrajectories()> _get_vehicle_trajectory_command_callback,
    std::function<std::vector<Visualization>()> _get_visualization_msgs_callback
)
:trajectoryCommand(_trajectoryCommand)
,get_vehicle_data(get_vehicle_data_callback)
,get_vehicle_trajectory_command_callback(_get_vehicle_trajectory_command_callback)
,get_visualization_msgs_callback(_get_visualization_msgs_callback)
{
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

        //std::cout << pan_x << " " << pan_y << " " << zoom << std::endl;

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


        // end path drawing mode
        if(!mouse_left_button)
        {
            trajectoryCommand->set_path(path_painting_in_progress_vehicle_id, path_painting_in_progress, 0);

            path_painting_in_progress.clear();
            path_painting_in_progress_vehicle_id = -1;
        }
        return true;
    });

    drawingArea->signal_motion_notify_event().connect([&](GdkEventMotion* event) {
        mouse_x = (event->x - pan_x) / zoom;
        mouse_y = -(event->y - pan_y) / zoom;

        vehicle_id_in_focus = find_vehicle_id_in_focus();


        // if in path drawing mode
        if(mouse_left_button)
        {
            if(is_valid_point_for_path(mouse_x, mouse_y))
            {
                // add new point to path
                path_painting_in_progress.emplace_back(mouse_x, mouse_y);
                assert(path_painting_in_progress.size() >= 2);

                auto p2 = path_painting_in_progress.at(path_painting_in_progress.size()-1);
                auto p1 = path_painting_in_progress.at(path_painting_in_progress.size()-2);
                path_painting_in_progress_yaw = atan2(p2.y-p1.y, p2.x-p1.x);
            }
        }

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

bool MapViewUi::is_valid_point_for_path(double x, double y)
{
    // Test if the point is inside a forward pointing cone, and at a particular distance.
    // This limits the maximum curvature of the path, and makes it driveable.

    if(path_painting_in_progress.empty()) return false;

    double dx = x - path_painting_in_progress.back().x;
    double dy = y - path_painting_in_progress.back().y;
    double dist_sq = dx*dx + dy*dy;
    if(dist_sq < (path_segment_length*path_segment_length)) return false;
    if(dist_sq > ((path_segment_length+0.1)*(path_segment_length+0.1))) return false;

    double c1 = cos(path_painting_in_progress_yaw + path_segment_max_angle);
    double s1 = sin(path_painting_in_progress_yaw + path_segment_max_angle);

    double c2 = cos(path_painting_in_progress_yaw - path_segment_max_angle);
    double s2 = sin(path_painting_in_progress_yaw - path_segment_max_angle);

    if(dx * s1 - dy * c1 < 0) return false;
    if(dx * s2 - dy * c2 > 0) return false;
    return true;
}



void MapViewUi::draw(const DrawingContext& ctx)
{
    ctx->save();
    {
        ctx->translate(pan_x, pan_y);
        ctx->scale(zoom, -zoom);

        draw_grid(ctx);

        // Draw vehicle focus disk
        if(vehicle_id_in_focus >= 0 && path_painting_in_progress_vehicle_id < 0)
        {
            ctx->set_source_rgba(0,0,1,0.4);
            ctx->arc(
                vehicle_data.at(vehicle_id_in_focus).at("pose_x")->get_latest_value(),
                vehicle_data.at(vehicle_id_in_focus).at("pose_y")->get_latest_value(),
                0.2, 0.0, 2 * M_PI
            );
            ctx->fill();
        }

        draw_received_trajectory_commands(ctx);

        draw_path_painting(ctx);

        draw_received_visualization_commands(ctx);

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

void MapViewUi::draw_received_trajectory_commands(const DrawingContext& ctx)
{
    VehicleTrajectories vehicleTrajectories = get_vehicle_trajectory_command_callback();


    for(const auto& entry : vehicleTrajectories) 
    {
        const auto vehicle_id = entry.first;
        const auto& trajectory_points = entry.second;

        std::vector<TrajectoryPoint> trajectory_segment;
        for (const auto& trajectory_point : trajectory_points)
        {
            trajectory_segment.push_back(trajectory_point.second);
        }        

        if(trajectory_segment.size() > 1)
        {
            // Draw trajectory interpolation
            for (int i = 2; i < int(trajectory_segment.size()); ++i)
            {
                const int n_interp = 20;
                ctx->set_source_rgb(0,0,0.8);
                ctx->move_to(trajectory_segment[i-1].px(), trajectory_segment[i-1].py());

                for (int interp_step = 1; interp_step < n_interp; ++interp_step)
                {
                    const uint64_t delta_t = 
                          trajectory_segment[i].t().nanoseconds() 
                        - trajectory_segment[i-1].t().nanoseconds();

                    TrajectoryInterpolation interp(
                        (delta_t * interp_step) / n_interp + trajectory_segment[i-1].t().nanoseconds(),  
                        trajectory_segment[i-1],  
                        trajectory_segment[i]
                    );
                    
                    ctx->line_to(interp.position_x,interp.position_y);
                }

                ctx->line_to(trajectory_segment[i].px(), trajectory_segment[i].py());
                ctx->set_line_width(0.01);
                ctx->stroke();
            }

            // Draw trajectory points
            for(size_t i = 1; i < trajectory_segment.size(); ++i)
            {
                ctx->set_source_rgb(0,0,0.8);
                ctx->arc(
                    trajectory_segment[i].px(),
                    trajectory_segment[i].py(),
                    0.02, 0.0, 2 * M_PI
                );
                ctx->fill();
            }
        }
    }
}

void MapViewUi::draw_path_painting(const DrawingContext& ctx)
{
    if(!path_painting_in_progress.empty() && path_painting_in_progress_vehicle_id >= 0)
    {
        // Draw cone of valid directions
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

        // Draw path lines
        for (size_t i = 0; i < path_painting_in_progress.size()-1; ++i)
        {
            ctx->set_source_rgb(0.6,0,0);
            ctx->move_to(path_painting_in_progress[i].x, path_painting_in_progress[i].y);
            ctx->line_to(path_painting_in_progress[i+1].x, path_painting_in_progress[i+1].y);
            ctx->set_line_width(0.01);
            ctx->stroke();
        }

        // Draw path nodes
        for (size_t i = 0; i < path_painting_in_progress.size(); ++i)
        {
            ctx->set_source_rgb(0.6,0,0);
            ctx->arc(path_painting_in_progress[i].x, path_painting_in_progress[i].y, 0.02, 0.0, 2 * M_PI);
            ctx->fill();
        }
    }
}

//Draw all received viz commands on the screen
void MapViewUi::draw_received_visualization_commands(const DrawingContext& ctx) {
    //Get commands
    std::vector<Visualization> visualization_commands = get_visualization_msgs_callback();

    for(const auto& entry : visualization_commands) 
    {
        if ((entry.type() == VisualizationType::LineStrips || entry.type() == VisualizationType::Polygon) 
            && entry.points().size() > 1) 
        {
            const auto& message_points = entry.points();

            //Set beginning point
            ctx->set_source_rgb(entry.color().r(), entry.color().g(), entry.color().b());
            ctx->move_to(message_points.at(0).x(), message_points.at(0).y());

            for (size_t i = 1; i < message_points.size(); ++i)
            {
                const auto& current_point = message_points.at(i);

                ctx->line_to(message_points.at(i).x(), message_points.at(i).y());
            }  

            //Line from end to beginning point to close the polygon
            if (entry.type() == VisualizationType::Polygon) {
                ctx->line_to(message_points.at(0).x(), message_points.at(0).y());
            }

            ctx->set_line_width(entry.size());
            ctx->stroke();      
        }
    }
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
            ctx->set_source_rgb(0,0,0);
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

        const double LF = 0.115;
        const double LR = 0.102;
        const double WH = 0.054;

        // Draw car image
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

        // Draw vehicle ID
        ctx->save();
        {
            ctx->translate(-0.03, 0);
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
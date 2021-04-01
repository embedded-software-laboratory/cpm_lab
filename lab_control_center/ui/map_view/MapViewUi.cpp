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

#include "MapViewUi.hpp"
#include <cassert>
#include <glibmm/main.h>
#include <libxml++-2.6/libxml++/libxml++.h>
#include <math.h>

#include "TrajectoryInterpolation.hpp"
#include "TrajectoryInterpolation.cxx"

#include "PathInterpolation.hpp"
#include "PathInterpolation.cxx"

#include <stdio.h>

/**
 * \file MapViewUi.cpp
 * \ingroup lcc_ui
 */

using namespace std::placeholders; //For std::bind

MapViewUi::MapViewUi(
    shared_ptr<TrajectoryCommand> _trajectoryCommand,
    shared_ptr<CommonRoadScenario> _commonroad_scenario,
    std::function<VehicleData()> get_vehicle_data_callback,
    std::function<VehicleTrajectories()> _get_vehicle_trajectory_command_callback,
    std::function<VehiclePathTracking()> _get_vehicle_path_tracking_command_callback,
    std::function<std::vector<CommonroadObstacle>()> _get_obstacle_data,
    std::function<std::vector<Visualization>()> _get_visualization_msgs_callback
)
:trajectoryCommand(_trajectoryCommand)
,commonroad_scenario(_commonroad_scenario)
,get_vehicle_data(get_vehicle_data_callback)
,get_vehicle_trajectory_command_callback(_get_vehicle_trajectory_command_callback)
,get_vehicle_path_tracking_command_callback(_get_vehicle_path_tracking_command_callback)
,get_visualization_msgs_callback(_get_visualization_msgs_callback)
,get_obstacle_data(_get_obstacle_data)
{
    //Create a drawing area to draw on (for showing vehicles, trajectories, obstacles etc.)
    drawingArea = Gtk::manage(new Gtk::DrawingArea());
    drawingArea->set_double_buffered();
    drawingArea->show();
    

    image_car = Cairo::ImageSurface::create_from_png("ui/map_view/car_small.png");
    image_object = Cairo::ImageSurface::create_from_png("ui/map_view/object_small.png");
    image_map = Cairo::ImageSurface::create_from_png("ui/map_view/map.png");
    //image_arrow = Cairo::ImageSurface::create_from_png("ui/map_view/arrow.png");
    image_labcam = Cairo::ImageSurface::create_from_png("ui/map_view/labcam.png");
    
    update_dispatcher.connect([&](){ 
        //Pan depending on key press
        if (key_up) pan_y += key_move;
        if (key_down) pan_y -= key_move;
        if (key_left) pan_x += key_move;
        if (key_right) pan_x -= key_move;

        vehicle_data = this->get_vehicle_data();
        drawingArea->queue_draw(); 
    });

    draw_loop_thread = std::thread([&](){
        while(1) {
            usleep(20000);
            update_dispatcher.emit();
        }
    });

    drawingArea->add_events(Gdk::SCROLL_MASK);
    drawingArea->add_events(Gdk::BUTTON_PRESS_MASK);
    drawingArea->add_events(Gdk::BUTTON_RELEASE_MASK);
    drawingArea->add_events(Gdk::POINTER_MOTION_MASK);
    drawingArea->add_events(Gdk::KEY_PRESS_MASK);
    drawingArea->add_events(Gdk::KEY_RELEASE_MASK);

    drawingArea->set_can_focus(true);

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

        //(Leons comment, not his code) In my understanding: In between scales [30, 900] zooming is allowed, and we can zoom in and out with the mouse wheel
        //These values are applied in our draw() function, which is called by a callback function of the drawingArea itself (so: handled by Gtk)
        if(zoom_speed != 1)
        {
            pan_x = event->x - zoom_speed * (event->x - pan_x);
            pan_y = event->y - zoom_speed * (event->y - pan_y);
            zoom *= zoom_speed;
        }

        //std::cout << pan_x << " " << pan_y << " " << zoom << std::endl;

        return true; 
    });

    //For moving the area with the arrow keys - the according values are 
    drawingArea->signal_key_press_event().connect([&] (GdkEventKey* event) {
        if (event->type == GDK_KEY_PRESS)
        {
            //Multiple keys may be pressed at once
            if (event->keyval == GDK_KEY_Up) key_up = true;
            if (event->keyval == GDK_KEY_Down) key_down = true;
            if (event->keyval == GDK_KEY_Left) key_left = true;
            if (event->keyval == GDK_KEY_Right) key_right = true;

            if (key_up || key_down || key_left || key_right) return true; //Signal was handled
        }
        return false; //Propagate signal
    }, false);

    drawingArea->signal_key_release_event().connect([&] (GdkEventKey* event) {
        if (event->type == GDK_KEY_RELEASE)
        {
            if (event->keyval == GDK_KEY_Up) key_up = false;
            if (event->keyval == GDK_KEY_Down) key_down = false;
            if (event->keyval == GDK_KEY_Left) key_left = false;
            if (event->keyval == GDK_KEY_Right) key_right = false;

            if (key_up || key_down || key_left || key_right) return true; //Signal was handled
        }
        return false; //Propagate signal
    }, false);

    drawingArea->signal_button_press_event().connect([&](GdkEventButton* event) {
        if(event->button == 1) mouse_left_button = true;
        if(event->button == 3) mouse_right_button = true;

        // start path drawing mode
        if(mouse_left_button)
        {
            //Get focus for key events
            drawingArea->grab_focus();

            path_painting_in_progress_vehicle_id = find_vehicle_id_in_focus();
            if(path_painting_in_progress_vehicle_id >= 0)
            {
                path_painting_in_progress.clear();
                auto& vehicle_timeseries = vehicle_data.at(path_painting_in_progress_vehicle_id);

                Pose2D pose;
                pose.x(vehicle_timeseries.at("pose_x")->get_latest_value());
                pose.y(vehicle_timeseries.at("pose_y")->get_latest_value());
                pose.yaw(vehicle_timeseries.at("pose_yaw")->get_latest_value());
                path_painting_in_progress.push_back(pose);
            }
        }
        else if (mouse_right_button)
        {
            //Get focus for key events
            drawingArea->grab_focus();

            old_event_x = event->x;
            old_event_y = event->y;
        }

        return true;
    });

    drawingArea->signal_button_release_event().connect([&](GdkEventButton* event) {
        if(event->button == 1) mouse_left_button = false;
        if(event->button == 3) mouse_right_button = false;

        // end path drawing mode
        if(!mouse_left_button)
        {
            if(path_painting_in_progress.size() > 25)
            {
                trajectoryCommand->set_path(path_painting_in_progress_vehicle_id, path_painting_in_progress);
            }

            path_painting_in_progress.clear();
            path_painting_in_progress_vehicle_id = -1;
        }
        return true;
    });

    drawingArea->signal_motion_notify_event().connect([&](GdkEventMotion* event) {
        // Transform mouse-event from canvas coordinates into world coordinates by reversing all steps done while drawing (compare (*))
        // Rotation around z-axis corresponds to the following matrix multiplication [x'] = [cos(a) -sin(a)] * [x]
        //                                                                           [y']   [sin(a)  cos(a)]   [y]
        double event_x =  ((event->x - pan_x) / zoom) - rotation_fixpoint_x;
        double event_y = -((event->y - pan_y) / zoom) - rotation_fixpoint_y;
        mouse_x =  (cos(-rotation)*event_x - sin(-rotation)*event_y) + rotation_fixpoint_x;
        mouse_y =  (sin(-rotation)*event_x + cos(-rotation)*event_y) + rotation_fixpoint_y;


        vehicle_id_in_focus = find_vehicle_id_in_focus();

        // if in path drawing mode
        if(mouse_left_button && !path_painting_in_progress.empty())
        {
            while(1)
            {
                assert(!path_painting_in_progress.empty());

                Pose2D current_pose = path_painting_in_progress.back();

                const double mouse_direction = atan2(
                    mouse_y - current_pose.y(),
                    mouse_x - current_pose.x()
                );

                double delta_yaw = mouse_direction - path_painting_in_progress.back().yaw();
                delta_yaw = remainder(delta_yaw, 2*M_PI);
                delta_yaw *= 0.1;
                delta_yaw = fmax(delta_yaw, -path_segment_max_delta_yaw);
                delta_yaw = fmin(delta_yaw,  path_segment_max_delta_yaw);

                Pose2D next_pose;
                next_pose.yaw(remainder(current_pose.yaw()+delta_yaw, 2*M_PI));
                next_pose.x(current_pose.x() + path_segment_delta_s * cos(next_pose.yaw()));
                next_pose.y(current_pose.y() + path_segment_delta_s * sin(next_pose.yaw()));


                const double dx1 = mouse_x - current_pose.x();
                const double dy1 = mouse_y - current_pose.y();

                const double dx2 = mouse_x - next_pose.x();
                const double dy2 = mouse_y - next_pose.y();

                const double dist1sq = dx1*dx1 + dy1*dy1;
                const double dist2sq = dx2*dx2 + dy2*dy2;


                if(dist1sq > dist2sq && dist2sq > 0.01)
                {
                    path_painting_in_progress.push_back(next_pose);
                }
                else
                {
                    break;
                }
            }
        }
        else if(!mouse_left_button && mouse_right_button)
        {
            double mouse_dist_x = (event->x - old_event_x);
            double mouse_dist_y = (event->y - old_event_y);

            pan_x += mouse_dist_x;
            pan_y += mouse_dist_y;

            old_event_x = event->x;
            old_event_y = event->y;
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

void MapViewUi::draw(const DrawingContext& ctx)
{
    ctx->save();
    {   
        // transforming (*)
        ctx->translate(pan_x, pan_y);
        ctx->scale(zoom, -zoom);
        
        // rotate mapview without changing the center of the map
        ctx->translate(rotation_fixpoint_x,rotation_fixpoint_y);
        ctx->rotate(rotation);
        ctx->translate(-rotation_fixpoint_x,-rotation_fixpoint_y);       

        //draw_grid(ctx);
        //Draw map
        if (commonroad_scenario)
        {
            commonroad_scenario->draw(ctx);
        }

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

        draw_lab_boundaries(ctx);

        draw_labcam(ctx);

        draw_received_trajectory_commands(ctx);

        draw_received_path_tracking_commands(ctx);

        for(const auto& entry : vehicle_data) {
            //const auto vehicle_id = entry.first;
            const auto& vehicle_timeseries = entry.second;

            if(vehicle_timeseries.at("pose_x")->has_new_data(1.0))
            {
                draw_vehicle_past_trajectory(ctx, vehicle_timeseries);
            }
        }

        draw_received_visualization_commands(ctx);

        draw_commonroad_obstacles(ctx);

        draw_path_painting(ctx);

        for(const auto& entry : vehicle_data) {
            const auto vehicle_id = entry.first;
            const auto& vehicle_timeseries = entry.second;

            if(vehicle_timeseries.at("pose_x")->has_new_data(1.0))
            {
                draw_vehicle_body(ctx, vehicle_timeseries, vehicle_id);
            }
        }
    }
    ctx->restore();
}

void MapViewUi::draw_lab_boundaries(const DrawingContext& ctx)
{
    ctx->save();

    ctx->set_line_width(0.005);
    ctx->set_source_rgb(77.0/255.0, 147.0/255.0, 215.0/255.0);

    auto lab_bound_x_1 = 0.0;
    auto lab_bound_x_2 = 4.5;
    auto lab_bound_y_1 = 0.0;
    auto lab_bound_y_2 = 4.0;

    //Move to first corner of lab boundaries
    ctx->move_to(lab_bound_x_1, lab_bound_y_1);

    //Draw lines
    ctx->line_to(lab_bound_x_1, lab_bound_y_2);
    ctx->line_to(lab_bound_x_2, lab_bound_y_2);
    ctx->line_to(lab_bound_x_2, lab_bound_y_1);
    ctx->line_to(lab_bound_x_1, lab_bound_y_1);
    ctx->stroke();

    //Show LCC boundaries text
    ctx->move_to(lab_bound_x_1, lab_bound_y_2);
    //Flip font
    Cairo::Matrix font_matrix(0.1, 0.0, 0.0, -0.1, 0.0, 0.0);
    ctx->set_font_matrix(font_matrix);
    //Draw text
    ctx->show_text("IPS boundary");

    ctx->restore();
}

void MapViewUi::draw_received_trajectory_commands(const DrawingContext& ctx)
{
    VehicleTrajectories vehicleTrajectories = get_vehicle_trajectory_command_callback();

    ctx->save();
    for(const auto& entry : vehicleTrajectories) 
    {
        //const auto vehicle_id = entry.first;
        const auto& trajectory = entry.second;

        rti::core::vector<TrajectoryPoint> trajectory_segment = trajectory.trajectory_points();
        
        if(trajectory_segment.size() < 2 ) continue;
        
        uint64_t t_now = cpm::get_time_ns();

        ctx->set_line_width(0.01);

        // Draw trajectory interpolation - use other color for already invalid parts (timestamp older than current point in time)
        // start from 1 because of i-1
        for (size_t i = 1; i < trajectory_segment.size(); ++i)
        {
            const int n_interp = 20;
            
            ctx->begin_new_path();
            ctx->move_to(trajectory_segment[i-1].px(),
                         trajectory_segment[i-1].py()
            );   
            for (int interp_step = 1; interp_step <= n_interp; ++interp_step)
            {
                const uint64_t delta_t = 
                        trajectory_segment[i].t().nanoseconds() 
                    - trajectory_segment[i-1].t().nanoseconds();

                const uint64_t t_cur = (delta_t * interp_step) / n_interp + trajectory_segment[i-1].t().nanoseconds();

                TrajectoryInterpolation interp(
                    t_cur,
                    trajectory_segment[i-1],  
                    trajectory_segment[i]
                );
                
                ctx->line_to(interp.position_x,
                             interp.position_y
                );

                if (t_cur < t_now)
                {
                    //Color for past segments
                    ctx->set_source_rgb(0.7,0.7,0.7);
                }
                else
                {
                    //Color for current and future segments
                    ctx->set_source_rgb(0,0,0.8);
                }

                ctx->stroke();
                ctx->move_to(interp.position_x,
                             interp.position_y
                );
            }
        }

        // Draw trajectory points
        ctx->begin_new_path();
        for(size_t i = 0; i < trajectory_segment.size(); ++i)
        {
            //Color based on future / current interpolation
            uint64_t t_cur = trajectory_segment.at(i).t().nanoseconds();
            if (t_cur < t_now)
            {
                ctx->set_source_rgb(0.7,0.7,0.7);
            }
            else
            {
                ctx->set_source_rgb(0,0,0.8);
            }

            ctx->arc(
                trajectory_segment[i].px(),
                trajectory_segment[i].py(),
                0.02, 0.0, 2 * M_PI
            );
            ctx->fill();
        }
    }
    ctx->restore();
}

void MapViewUi::draw_received_path_tracking_commands(const DrawingContext& ctx)
{
    VehiclePathTracking vehiclePathTracking = get_vehicle_path_tracking_command_callback();

    ctx->save();
    for(const auto& entry : vehiclePathTracking) 
    {
        const auto& command = entry.second;

        rti::core::vector<PathPoint> path = command.path();
        
        if(path.size() < 2 ) continue;

        ctx->set_line_width(0.01);

        // Draw trajectory interpolation - use other color for already invalid parts (timestamp older than current point in time)
        // start from 1 because of i-1
        for (size_t i = 1; i < path.size(); ++i)
        {
            const int n_interp = 20;
            
            ctx->begin_new_path();
            ctx->move_to(path[i-1].pose().x(),
                         path[i-1].pose().y()
            );   

            double start = path[i-1].s();
            double end = path[i].s();
            double ds = (end - start) / n_interp;
            double s_query = start;

            for (int j = 0; j < n_interp; j++)
            {
                s_query += ds;

                // calculate distance to reference path
                PathInterpolation path_interpolation(
                    s_query, path[i-1], path[i]
                );
                
                ctx->line_to(path_interpolation.position_x,
                             path_interpolation.position_y
                );

                ctx->set_source_rgb(0,0.8,0.8);

                ctx->stroke();
                ctx->move_to(path_interpolation.position_x,
                             path_interpolation.position_y
                );
            }
        }

        // Draw path points
        ctx->begin_new_path();
        for(size_t i = 0; i < path.size(); ++i)
        {
            ctx->set_source_rgb(0,0.8,0.8);

            ctx->arc(
                path[i].pose().x(),
                path[i].pose().y(),
                0.02, 0.0, 2 * M_PI
            );
            ctx->fill();
        }
    }

    ctx->restore();
}

void MapViewUi::draw_path_painting(const DrawingContext& ctx)
{
    if(path_painting_in_progress.size() > 1 && path_painting_in_progress_vehicle_id >= 0)
    {
        // Draw path lines
        ctx->set_source_rgb(0.6,0,0);
        ctx->move_to(path_painting_in_progress[0].x(), path_painting_in_progress[0].y());
        for (size_t i = 1; i < path_painting_in_progress.size(); ++i)
        {
            ctx->line_to(path_painting_in_progress[i].x(), path_painting_in_progress[i].y());
        }
        ctx->set_line_width(0.01);
        ctx->stroke();
    }
}

/**
 * \brief Determine the offset for alligned string messages drawn draw_received_visualization_command()
 * \param ext TODO
 * \param anchor TODO
 * \param offs_x TODO
 * \param offs_y TODO
 * \ingroup lcc_ui
 */
void get_text_offset(Cairo::TextExtents ext, StringMessageAnchor anchor, double& offs_x, double& offs_y)
{
    // x offset
    switch(anchor.underlying())
    {
        case StringMessageAnchor::TopRight:
        case StringMessageAnchor::CenterRight:
        case StringMessageAnchor::BottomRight:
            // substract bearing twice so the gap between anchor point and text
            // behaves equally as with a left sided anchor
            offs_x = -ext.width - 2*ext.x_bearing;
            break;
        
        case StringMessageAnchor::TopCenter:
        case StringMessageAnchor::Center:
        case StringMessageAnchor::BottomCenter:
            
            offs_x = -ext.width/2 - ext.x_bearing;
            break;
        
        default:
            offs_x = 0.0;
    }
    // y offset
    switch(anchor.underlying())
    {
        case StringMessageAnchor::TopLeft:
        case StringMessageAnchor::TopCenter:
        case StringMessageAnchor::TopRight:
            
            offs_y = ext.y_bearing;
            break;
        
        case StringMessageAnchor::CenterLeft:
        case StringMessageAnchor::Center:
        case StringMessageAnchor::CenterRight:
            
            offs_y = ext.height/2 + ext.y_bearing;
            break;
        
        default:
            offs_y = 0.0;
    }
}


//Draw all received viz commands on the screen
void MapViewUi::draw_received_visualization_commands(const DrawingContext& ctx) {
    //Get commands
    std::vector<Visualization> visualization_commands = get_visualization_msgs_callback();

    for(const auto& entry : visualization_commands) 
    {
        if ((entry.type() == VisualizationType::LineStrips || 
             entry.type() == VisualizationType::Polygon    ||
             entry.type() == VisualizationType::FilledCircle )
            && entry.points().size() > 0)
        {
            const auto& message_points = entry.points();

            //Set beginning point
            ctx->set_source_rgb(entry.color().r()/255.0, entry.color().g()/255.0, entry.color().b()/255.0);
            ctx->move_to(message_points.at(0).x(), message_points.at(0).y());

            if(entry.type() == VisualizationType::FilledCircle)
            {
                const auto& radius = entry.size();
                ctx->arc(message_points.at(0).x(), message_points.at(0).y(), radius, 0.0, 2.0 * M_PI);
                ctx->fill(); // replaces stroke()
            }
            else if(entry.points().size() < 2) // type definitely is LineStrips or Polygon
            {
                cpm::Logging::Instance().write(1, "%s", "WARNING: Visualisation of Polygon or LineStrips with < 2 points");
            }
            else
            {
                for (size_t i = 1; i < message_points.size(); ++i)
                {
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
        else if (entry.type() == VisualizationType::StringMessage
                 && entry.string_message().size() > 0 && entry.points().size() >= 1) {
            
            ctx->save();
            // ctx->rotate(-rotation);
            //Set font properties
            ctx->set_source_rgb(entry.color().r()/255.0, entry.color().g()/255.0, entry.color().b()/255.0);
            ctx->set_font_size(entry.size());

            //Align
            Cairo::TextExtents ext;
            ctx->get_text_extents(entry.string_message(), ext);
            
            // Firstly, compute text offset neglecting the current map view rotation.
            // Secondly, apply rotation matrix to text_offset so that offset is correclty applied dependent on the map view rotation.
            double text_offset_left, text_offset_right;
            get_text_offset(ext, entry.string_message_anchor(), text_offset_left, text_offset_right);
            double text_offset_x = (cos(-rotation)*text_offset_left - sin(-rotation)*text_offset_right);
            double text_offset_y = (sin(-rotation)*text_offset_left + cos(-rotation)*text_offset_right);

            // Move to the correct position and rotate so that text is shown horizontally
            ctx->move_to(entry.points().at(0).x() + text_offset_x, 
                         entry.points().at(0).y() + text_offset_y );
            ctx->rotate(-rotation);

            //Flip font
            Cairo::Matrix font_matrix(entry.size(), 0.0, 0.0, -1.0 * entry.size(), 0.0, 0.0);
            ctx->set_font_matrix(font_matrix);

            //Draw text
            ctx->show_text(entry.string_message().c_str());

            ctx->restore();
        }
    }
}

/**
 * \brief Print XML Node content
 * \param node XML Node 
 * \param indentation Indentation for printing the XML Node
 * \ingroup lcc_ui
 */
void print_node(const xmlpp::Node* node, unsigned int indentation = 0)
{
  const Glib::ustring indent(indentation, ' ');
  std::cout << std::endl; //Separate nodes by an empty line.

  const auto nodeContent = dynamic_cast<const xmlpp::ContentNode*>(node);
  const auto nodeText = dynamic_cast<const xmlpp::TextNode*>(node);
  const auto nodeComment = dynamic_cast<const xmlpp::CommentNode*>(node);

  if(nodeText && nodeText->is_white_space()) //Let's ignore the indenting - you don't always want to do this.
    return;

  const auto nodename = node->get_name();

  if(!nodeText && !nodeComment && !nodename.empty()) //Let's not say "name: text".
  {
    const auto namespace_prefix = node->get_namespace_prefix();

    std::cout << indent << "Node name = " ;
    if(!namespace_prefix.empty())
      std::cout << namespace_prefix << ":";
    std::cout << nodename << std::endl;
  }
  else if(nodeText) //Let's say when it's text. - e.g. let's say what that white space is.
  {
    std::cout << indent << "Text Node" << std::endl;
  }

  //Treat the various node types differently:
  if(nodeText)
  {
    std::cout << indent << "text = \"" << (nodeText->get_content()) << "\"" << std::endl;
  }
  else if(nodeComment)
  {
    std::cout << indent << "comment = " << (nodeComment->get_content()) << std::endl;
  }
  else if(nodeContent)
  {
    std::cout << indent << "content = " << (nodeContent->get_content()) << std::endl;
  }
  else if(const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node))
  {
    //A normal Element node:

    //line() works only for ElementNodes.
    std::cout << indent << "     line = " << node->get_line() << std::endl;

    //Print attributes:
    for (const auto& attribute : nodeElement->get_attributes())
    {
      const auto namespace_prefix = attribute->get_namespace_prefix();

      std::cout << indent << "  Attribute ";
      if(!namespace_prefix.empty())
        std::cout << (namespace_prefix) << ":";
      std::cout << (attribute->get_name()) << " = "
                << (attribute->get_value()) << std::endl;
    }

    const auto attribute = nodeElement->get_attribute("title");
    if(attribute)
    {
      std::cout << indent;
      if (dynamic_cast<const xmlpp::AttributeNode*>(attribute))
        std::cout << "AttributeNode ";
      else if (dynamic_cast<const xmlpp::AttributeDeclaration*>(attribute))
        std::cout << "AttributeDeclaration ";
      std::cout << "title = " << (attribute->get_value()) << std::endl;
    }
  }

  if(!nodeContent)
  {
    //Recurse through child nodes:
    for(const auto& child : node->get_children())
    {
      print_node(child, indentation + 2); //recursive
    }
  }
}

void MapViewUi::draw_grid(const DrawingContext& ctx)
{
    // Draw map (roads) image     
    
    std::string filepath = "./ui/map_view/LabMapCommonRoad.xml";

    xmlpp::DomParser parser;
    vector<double> lanelet_x;
    vector<double> lanelet_y;

    try
    {
        parser.parse_file(filepath);

        if(!parser) 
        {
            cpm::Logging::Instance().write(
                1,
                "%s", 
                "ERROR: can not parse file"
            );
        }
        const auto pNode = parser.get_document()->get_root_node(); //deleted by DomParser.
        print_node(pNode);

    }
    catch(const std::exception& ex)
    {
        std::cerr << "Exception caught: " << ex.what() << std::endl;
    }


    /*for (size_t i = 1; i < lanelet_x.size(); ++i)
    {
        if(i == 1) ctx->move_to(lanelet_x[i], lanelet_y[i]);
        else ctx->line_to(lanelet_x[i], lanelet_y[i]);
    }
    ctx->set_source_rgb(1,0,0);
    ctx->set_line_width(0.01);
    ctx->stroke();
    ctx->restore();*/

    // Draw grid lines
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


void MapViewUi::draw_labcam(const DrawingContext& ctx)
{
    ctx->save();
    {
        const double scale = 0.1/image_labcam->get_width();
        ctx->translate(-0.1,1.95);
        ctx->rotate(M_PI / 2);
        ctx->scale(scale, scale);
        ctx->set_source(image_labcam,0,0);
        ctx->paint();
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
        //const double WH = 0.054;

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
            ctx->rotate(-yaw - rotation);
            ctx->scale(scale, -scale);
            ctx->move_to(0,0);
            Cairo::TextExtents extents;
            ctx->get_text_extents(to_string(static_cast<int>(vehicle_id)), extents); //Need to cast, else uint8_t is interpreted not as number, but as symbol

            ctx->move_to(-extents.width/2 - extents.x_bearing, -extents.height/2 - extents.y_bearing);
            ctx->set_source_rgb(1,1,1);
            ctx->show_text(to_string(static_cast<int>(vehicle_id)));

            ctx->move_to(-extents.width/2 - extents.x_bearing - 0.6, -extents.height/2 - extents.y_bearing - 0.4);
            ctx->set_source_rgb(1,.1,.1);
            ctx->show_text(to_string(static_cast<int>(vehicle_id)));
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

void MapViewUi::draw_vehicle_shape(const DrawingContext& ctx, CommonroadDDSShape& shape)
{
    ctx->save();
    ctx->set_line_width(0.005);

    for (auto circle : shape.circles())
    {
        ctx->save();

        //Move to center
        ctx->move_to(circle.center().x(), circle.center().y());

        //Draw circle
        ctx->arc(circle.center().x(), circle.center().y(), circle.radius(), 0.0, 2 * M_PI);
        ctx->stroke();

        ctx->restore();
    }

    for (auto polygon : shape.polygons())
    {
        if (polygon.points().size() < 3)
        {
            std::cerr << "Points missing in translated polygon (at least 3 required) - will not be drawn" << std::endl;
            LCCErrorLogger::Instance().log_error("Points missing in translated polygon (at least 3 required) - will not be drawn");
        }
        else
        {
            ctx->save();

            //Move to first point
            ctx->move_to(polygon.points().at(0).x(), polygon.points().at(0).y());

            //Draw lines to remaining points
            for (auto& point : polygon.points())
            {
                ctx->line_to(point.x(), point.y());
            }
            //Finish polygon by drawing a line to the starting point
            ctx->line_to(polygon.points().at(0).x(), polygon.points().at(0).y());
            ctx->fill_preserve();
            ctx->stroke();

            ctx->restore();
        }
    }

    for (auto rectangle : shape.rectangles())
    {
        ctx->save();

        //Translate to center of object
        ctx->translate(rectangle.center().x(), rectangle.center().y());

        //Rotate, if necessary
        ctx->rotate(rectangle.orientation());

        auto length = rectangle.length();
        auto width = rectangle.width();

        //Move to first corner from center
        ctx->move_to((- (length/2)), (- (width/2)));

        //Draw lines
        ctx->line_to((- (length/2)), (  (width/2)));
        ctx->line_to((  (length/2)), (  (width/2)));
        ctx->line_to((  (length/2)), (- (width/2)));
        ctx->line_to((- (length/2)), (- (width/2)));
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
    }

    //TODO: Improve shape drawing
    //For example: Color coding instead of longer names (e.g. for (non-)moving objects)

    ctx->restore();
}

std::pair<double, double> MapViewUi::get_shape_center(CommonroadDDSShape& shape)
{
    double x, y = 0.0;
    double center_count = 0.0;

    for (auto circle : shape.circles())
    {
        auto center = circle.center();
        x += center.x();
        y += center.y();
        ++center_count;
    }

    for (auto polygon : shape.polygons())
    {
        if (polygon.points().size() > 0)
        {
            double sum_x = 0;
            double sum_y = 0;

            for (auto point : polygon.points())
            {
                sum_x += point.x();
                sum_y += point.y();
            }
            
            x += sum_x / static_cast<double>(polygon.points().size());
            y += sum_y / static_cast<double>(polygon.points().size());
            ++center_count;
        }
    }

    for (auto rectangle : shape.rectangles())
    {
        x += rectangle.center().x();
        y += rectangle.center().y();
        ++center_count;
    }

    if (center_count > 0)
    {
        x /= center_count;
        y /= center_count;
    }

    return std::pair<double, double>(x, y);
}

void MapViewUi::draw_commonroad_obstacles(const DrawingContext& ctx)
{
    //Behavior is currently similar to drawing a vehicle - TODO: Improve this later on    
    ctx->set_source_rgb(1,.5,.1);

    assert(get_obstacle_data);
    for (auto entry : get_obstacle_data())
    {
        ctx->save();

        const double x = entry.pose().x();
        const double y = entry.pose().y();
        const double yaw = entry.pose().yaw();

        ctx->translate(x,y);
        ctx->rotate(yaw);

        // const double LF = 0.115;
        // const double LR = 0.102;
        //const double WH = 0.054;

        // Draw car image (TODO: Change this later, e.g. to shape)
        ctx->save();
        {
            // const double scale = 0.224/image_object->get_width();
            // ctx->translate( (LF+LR)/2-LR ,0);
            // ctx->scale(scale, scale);
            // ctx->translate(-image_object->get_width()/2, -image_object->get_height()/2);
            // ctx->set_source(image_object,0,0);
            // ctx->paint();
            //Make vehicle a bit transparent if the position is not exact
            if (! entry.pose_is_exact())
            {
                ctx->set_source_rgba(.7,.2,.7,.2); //Color used for inexact values
            }

            draw_vehicle_shape(ctx, entry.shape());
        }
        ctx->restore();

        //Draw description
        assert(commonroad_scenario->get_draw_configuration());
        ctx->save();
        if (commonroad_scenario->get_draw_configuration()->draw_obstacle_description.load()) {
            //Translate to shape center, if position is mostly defined by the shape's positional values
            auto shape_center = get_shape_center(entry.shape());
            ctx->translate(shape_center.first, shape_center.second);

            //Craft description from object properties
            std::stringstream description_stream;
            if (entry.pose_is_exact())
            {
                description_stream << "E,";
            }
            else
            {
                description_stream << "I,";
            }
            if (entry.is_moving())
            {
                description_stream << "M,";
            }
            else
            {
                description_stream << "S,";
            }
            switch(entry.type().underlying())
            {
                case ObstacleType::Unknown:
                    description_stream << "Unk: ";
                    break;
                case ObstacleType::Car: 
                    description_stream << "Car: ";
                    break;
                case ObstacleType::Truck:
                    description_stream << "Truck: ";
                    break;
                case ObstacleType::Bus:
                    description_stream << "Bus: ";
                    break;
                case ObstacleType::Motorcycle:
                    description_stream << "MCycle: ";
                    break;
                case ObstacleType::Bicycle:
                    description_stream << "BCycle: ";
                    break;
                case ObstacleType::Pedestrian:
                    description_stream << "Ped: ";
                    break;
                case ObstacleType::PriorityVehicle:
                    description_stream << "Prio: ";
                    break;
                case ObstacleType::Train:
                    description_stream << "Train: ";
                    break;
                case ObstacleType::ConstructionZone:
                    description_stream << "Constr: ";
                    break;
                case ObstacleType::ParkedVehicle:
                    description_stream << "Parked: ";
                    break;
                case ObstacleType::RoadBoundary:
                    description_stream << "Boundary: ";
                    break;
                default:
                    description_stream << "TODO: ";
                    break;
            }
            description_stream << static_cast<int>(entry.vehicle_id()); //CO for CommonroadObstacle

            ctx->translate(-0.03, 0);
            const double scale = 0.01;
            ctx->rotate(-yaw - rotation);
            ctx->scale(scale, -scale);
            ctx->move_to(0,0);
            Cairo::TextExtents extents;
            ctx->get_text_extents(description_stream.str(), extents);

            ctx->move_to(-extents.width/2 - extents.x_bearing, -extents.height/2 - extents.y_bearing);
            ctx->set_source_rgb(.1,.1,.1);
            ctx->show_text(description_stream.str());

            ctx->move_to(-extents.width/2 - extents.x_bearing - 0.6, -extents.height/2 - extents.y_bearing - 0.4);
            ctx->set_source_rgb(.1,.9,.1);
            ctx->show_text(description_stream.str());
        }

        ctx->restore();
        ctx->restore();
    }
}

Gtk::DrawingArea* MapViewUi::get_parent()
{
    return drawingArea;
}


void MapViewUi::rotate_by(double rotation) {
    this->rotation = std::fmod(this->rotation + (rotation * M_PI / 180), 2*M_PI);
}
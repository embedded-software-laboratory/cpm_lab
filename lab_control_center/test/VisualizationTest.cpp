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

#include <chrono>
#include <iostream>
#include <thread>

#include <dds/pub/ddspub.hpp>

#include "cpm/init.hpp"
#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/get_time_ns.hpp"
#include "Color.hpp"
#include "Visualization.hpp"

//In this test scenario, the timers are not stopped by the program but by the LCC stop signal

int main(int argc, char *argv[]) {
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("Logger_test");

    std::cout << "Creating visualization sender..." << std::endl;

    dds::pub::DataWriter<Visualization> viz_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<Visualization>("visualization"), dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable());

    // LineStrips
    Visualization viz;
    viz.id(1);
    viz.type(VisualizationType::LineStrips);
    viz.time_to_live(3000000000);
    viz.size(1.0);

    Point2D point1(0.0, 0.0);
    Point2D point2(1.0, 1.0);
    Point2D point3(2.0, -2.0);
    Point2D point4(3.0, 3.0);
    Point2D point5(4.0, -4.0);
    std::vector<Point2D> viz_points {point1, point2, point3, point4, point5};
    viz.points(rti::core::vector<Point2D>(viz_points));

    Color viz_color(255, 0, 255, 255);
    viz.color(viz_color);

    usleep(100000);

    std::cout << "Sending visualization line..." << std::endl;

    viz_writer.write(viz);

    // Polygon
    Visualization viz2;
    viz2.id(2);
    viz2.type(VisualizationType::Polygon);
    viz2.time_to_live(5000000000);
    viz2.size(0.05);

    Point2D point1_2(0.0, 0.0);
    Point2D point2_2(0.5, 0.5);
    Point2D point3_2(1.0, 0.0);
    std::vector<Point2D> viz2_points {point1_2, point2_2, point3_2};
    viz2.points(rti::core::vector<Point2D>(viz2_points));

    Color viz2_color(255, 255, 0, 255);
    viz2.color(viz2_color);

    usleep(100000);

    std::cout << "Sending visualization polygon..." << std::endl;

    viz_writer.write(viz2);

    // default StringMessage
    Visualization viz3;
    viz3.id(3);
    viz3.type(VisualizationType::StringMessage);
    viz3.time_to_live(10000000000);
    viz3.size(1.0);

    Point2D point1_3(0.2, 0.2);
    std::vector<Point2D> viz3_points {point1_3};
    viz3.points(rti::core::vector<Point2D>(viz3_points));

    Color viz3_color(255, 255, 255, 0);
    viz3.color(viz3_color);

    viz3.string_message("Hello LCC!");

    usleep(100000);

    std::cout << "Sending visualization string..." << std::endl;

    viz_writer.write(viz3);

    // FilledCircle (represents fix point of the alligned strings)
    Visualization viz4;
    viz4.id(4);
    viz4.type(VisualizationType::FilledCircle);
    viz4.time_to_live(11500000000);
    viz4.size(0.04);

    std::vector<Point2D> viz4_points{ Point2D(1.0, 3.0) };
    viz4.points(rti::core::vector<Point2D>(viz4_points));

    Color viz4_color(100, 255, 100, 0);
    viz4.color(viz4_color);

    usleep(100000);

    std::cout << "Sending visualisation filled circle..." << std::endl;
    
    viz_writer.write(viz4);

    // StringMessage with alignment
    Visualization viz5;
    viz5.id(5);
    viz5.type(VisualizationType::StringMessage);
    viz5.time_to_live(1200000000); // 1.5 seconds
    viz5.size(0.2);

    Point2D point1_5(1.0, 3.0);
    std::vector<Point2D> viz5_points {point1_5};
    viz5.points(rti::core::vector<Point2D>(viz5_points));

    Color viz5_color(255, 0, 128, 64);
    viz5.color(viz5_color);

    viz5.string_message("H - Default alignment");

    usleep(100000);

    std::cout << "Sending visualization string..." << std::endl;

    viz_writer.write(viz5);
    
    viz5.string_message("HXH");
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::BottomLeft);
    usleep(100000);
    std::cout << "Sending visualization BottomLeft string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::CenterLeft);
    usleep(100000);
    std::cout << "Sending visualization CenterLeft string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::TopLeft);
    usleep(100000);
    std::cout << "Sending visualization TopLeft string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::TopCenter);
    usleep(100000);
    std::cout << "Sending visualization TopCenter string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::TopRight);
    usleep(100000);
    std::cout << "Sending visualization TopRight string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::CenterRight);
    usleep(100000);
    std::cout << "Sending visualization CenterRight string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::BottomRight);
    usleep(100000);
    std::cout << "Sending visualization BottomLeft string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::BottomCenter);
    usleep(100000);
    std::cout << "Sending visualization BottomCenter string..." << std::endl;
    viz_writer.write(viz5);
    usleep(1000000);
    viz5.string_message_anchor(StringMessageAnchor::Center);
    usleep(100000);
    std::cout << "Sending visualization Center string..." << std::endl;
    viz_writer.write(viz5);
    

    std::cout << "Shutting down..." << std::endl;

    return 0;
}
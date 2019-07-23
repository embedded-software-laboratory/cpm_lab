#include <chrono>
#include <iostream>
#include <thread>

#include <dds/pub/ddspub.hpp>

#include "cpm/Logging.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "Visualization.hpp"

//In this test scenario, the timers are not stopped by the program but by the LCC stop signal

int main(int argc, char *argv[]) {
    cpm::Logging::Instance().set_id("Logger_test");

    std::cout << "Creating visualization sender..." << std::endl;

    dds::pub::DataWriter<Visualization> viz_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<Visualization>("visualization"), dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable());

    Visualization viz;
    viz.id(1);
    viz.type(VisualizationType::LineStrips);
    viz.action(VisualizationAction::ADD);
    viz.size(1.0);

    VisualizationPose point1(0.0, 0.0);
    VisualizationPose point2(1.0, 1.0);
    VisualizationPose point3(2.0, -2.0);
    VisualizationPose point4(3.0, 3.0);
    VisualizationPose point5(4.0, -4.0);
    std::vector<VisualizationPose> viz_points {point1, point2, point3, point4, point5};
    viz.points(rti::core::vector<VisualizationPose>(viz_points));

    Color viz_color(0.0, 1.0, 1.0);
    viz.color(viz_color);

    usleep(100000);

    std::cout << "Sending visualization line..." << std::endl;

    viz_writer.write(viz);

    Visualization viz2;
    viz2.id(2);
    viz2.type(VisualizationType::Polygon);
    viz2.action(VisualizationAction::ADD);
    viz2.size(0.05);

    VisualizationPose point1_2(0.0, 0.0);
    VisualizationPose point2_2(0.5, 0.5);
    VisualizationPose point3_2(1.0, 0.0);
    std::vector<VisualizationPose> viz2_points {point1_2, point2_2, point3_2};
    viz2.points(rti::core::vector<VisualizationPose>(viz2_points));

    Color viz2_color(1.0, 0.0, 1.0);
    viz2.color(viz2_color);

    usleep(100000);

    std::cout << "Sending visualization polygon..." << std::endl;

    viz_writer.write(viz2);

    Visualization viz3;
    viz3.id(3);
    viz3.type(VisualizationType::StringMessage);
    viz3.action(VisualizationAction::ADD);
    viz3.size(1.0);

    VisualizationPose point1_3(0.2, 0.2);
    std::vector<VisualizationPose> viz3_points {point1_3};
    viz3.points(rti::core::vector<VisualizationPose>(viz3_points));

    Color viz3_color(1.0, 1.0, 0.0);
    viz3.color(viz3_color);

    viz3.string_message("Hello LCC!");

    usleep(100000);

    std::cout << "Sending visualization string..." << std::endl;

    viz_writer.write(viz3);

    std::cout << "Shutting down..." << std::endl;

    return 0;
}
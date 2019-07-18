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

    std::cout << "Sending visualization..." << std::endl;

    viz_writer.write(viz);

    std::cout << "Shutting down..." << std::endl;

    return 0;
}
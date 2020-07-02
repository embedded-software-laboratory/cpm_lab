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

#include "defaults.hpp"
#include "stdio.h"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "ObstacleAggregator.hpp"
#include "TimeSeriesAggregator.hpp"
#include "HLCReadyAggregator.hpp"
#include "ObstacleSimulationManager.hpp"
#include "VisualizationCommandsAggregator.hpp"
#include "VehicleManualControl.hpp"
#include "VehicleAutomatedControl.hpp"
#include "ui/commonroad/CommonroadViewUI.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/map_view/MapViewUi.hpp"
#include "ui/right_tabs/TabsViewUI.hpp"
#include "ui/params/ParamViewUI.hpp"
#include "ui/timer/TimerViewUI.hpp"
#include "ui/logger/LoggerViewUI.hpp"
#include "ui/setup/SetupViewUI.hpp"
#include "LogStorage.hpp"
#include "ParameterServer.hpp"
#include "ParameterStorage.hpp"
#include "TrajectoryCommand.hpp"
#include "ui/MainWindow.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "TimerTrigger.hpp"
#include "cpm/init.hpp"

#include "commonroad_classes/CommonRoadScenario.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <functional>
#include <sstream>

//For exit handlers
#include <signal.h>
#include <stdlib.h>
#include <cstdlib>

using namespace std::placeholders;

void deploy_cloud_discovery() {
    std::string command = "tmux new-session -d -s \"rticlouddiscoveryservice\" \"rticlouddiscoveryservice -transport 25598\"";
    system(command.c_str());
}

void kill_cloud_discovery() {
    std::string command = "tmux kill-session -t \"rticlouddiscoveryservice\"";
    system(command.c_str());
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

void interrupt_handler(int s) {
    kill_cloud_discovery();
    exit(1);
}

#pragma GCC diagnostic pop

void exit_handler() {
    kill_cloud_discovery();
}

int main(int argc, char *argv[])
{
    //Must be done first, s.t. no class using the logger produces an error
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("lab_control_center");

    //Create regular and irregular (interrupt) exit handlers for IPS and Cloud Discovery Service
    struct sigaction interruptHandler;
    interruptHandler.sa_handler = interrupt_handler;
    sigemptyset(&interruptHandler.sa_mask);
    interruptHandler.sa_flags = 0;
    sigaction(SIGINT, &interruptHandler, NULL);

    std::atexit(exit_handler);

    //Start IPS and Cloud Discovery Service which are always required to communicate with real vehicles
    deploy_cloud_discovery();

    //Read command line parameters (current params: auto_start and config_file)
    //TODO auto_start: User does not need to trigger the process manually / does not need to press 'start' when all participants are ready

    std::string config_file = cpm::cmd_parameter_string("config_file", "parameters.yaml", argc, argv);

    //Load commonroad scenario (TODO: Implement load by user, this is just a test load)
    std::string filepath_2018 = "./ui/map_view/LabMapCommonRoad.xml";
    std::string filepath_2020 = "./ui/map_view/LabMapCommonRoad.xml";
    std::string filepath_parked_vehicles = "./ui/map_view/LabMapCommonRoad.xml";
    std::string filepath_occupancy = "./ui/map_view/LabMapCommonRoad.xml";
    auto commonroad_scenario = std::make_shared<CommonRoadScenario>();
    try
    {
        commonroad_scenario->load_file(filepath_2018);
        //commonroad_scenario->transform_coordinate_system(0.5, 0.0, -4.0);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    auto storage = make_shared<ParameterStorage>(config_file, 32);
    ParameterServer server(storage);
    storage->register_on_param_changed_callback(std::bind(&ParameterServer::resend_param_callback, &server, _1));

    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
    Glib::RefPtr<Gtk::CssProvider> cssProvider = Gtk::CssProvider::create();
    cssProvider->load_from_path("ui/style.css");
    Gtk::StyleContext::create()->add_provider_for_screen (Gdk::Display::get_default()->get_default_screen(),cssProvider,500);

    bool use_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    auto obstacle_simulation_manager = std::make_shared<ObstacleSimulationManager>(commonroad_scenario, use_simulated_time);

    auto timerTrigger = make_shared<TimerTrigger>(use_simulated_time);
    auto timerViewUi = make_shared<TimerViewUI>(timerTrigger);
    auto logStorage = make_shared<LogStorage>();
    auto loggerViewUi = make_shared<LoggerViewUI>(logStorage);
    auto vehicleManualControl = make_shared<VehicleManualControl>();
    auto vehicleAutomatedControl = make_shared<VehicleAutomatedControl>();
    auto trajectoryCommand = make_shared<TrajectoryCommand>();
    auto timeSeriesAggregator = make_shared<TimeSeriesAggregator>();
    auto obstacleAggregator = make_shared<ObstacleAggregator>(commonroad_scenario); //Use scenario to register reset callback if scenario is reloaded
    auto hlcReadyAggregator = make_shared<HLCReadyAggregator>();
    auto visualizationCommandsAggregator = make_shared<VisualizationCommandsAggregator>();
    unsigned int cmd_domain_id = cpm::cmd_parameter_int("dds_domain", 0, argc, argv);
    std::string cmd_dds_initial_peer = cpm::cmd_parameter_string("dds_initial_peer", "", argc, argv);
    //Create deploy class
    std::shared_ptr<Deploy> deploy_functions = std::make_shared<Deploy>(
        cmd_domain_id, 
        cmd_dds_initial_peer, 
        [&](uint8_t id){vehicleAutomatedControl->stop_vehicle(id);
    });
    auto mapViewUi = make_shared<MapViewUi>(
        trajectoryCommand, 
        commonroad_scenario,
        [=](){return timeSeriesAggregator->get_vehicle_data();},
        [=](){return timeSeriesAggregator->get_vehicle_trajectory_commands();},
        [=](){return obstacleAggregator->get_obstacle_data();}, 
        [=](){return visualizationCommandsAggregator->get_all_visualization_messages();}
    );
    auto monitoringUi = make_shared<MonitoringUi>(
        deploy_functions, 
        [=](){return timeSeriesAggregator->get_vehicle_data();}, 
        [=](){return hlcReadyAggregator->get_hlc_ids_string();},
        [=](){return timeSeriesAggregator->get_vehicle_trajectory_commands();},
        [=](){return timeSeriesAggregator->reset_all_data();}
    );
    auto vehicleManualControlUi = make_shared<VehicleManualControlUi>(vehicleManualControl);
    auto paramViewUi = make_shared<ParamViewUI>(storage, 5);
    auto commonroadViewUi = make_shared<CommonroadViewUI>(commonroad_scenario);
    auto setupViewUi = make_shared<SetupViewUI>(
        deploy_functions,
        vehicleAutomatedControl, 
        obstacle_simulation_manager,
        [=](){return hlcReadyAggregator->get_hlc_ids_uint8_t();}, 
        [=](){return timeSeriesAggregator->get_vehicle_data();},
        [=](bool simulated_time, bool reset_timer){return timerViewUi->reset(simulated_time, reset_timer);}, 
        [=](){return timeSeriesAggregator->reset_all_data();}, 
        [=](){return obstacleAggregator->reset_all_data();}, 
        [=](){return trajectoryCommand->stop_all();}, 
        [=](){return monitoringUi->reset_vehicle_view();}, 
        [=](){return visualizationCommandsAggregator->reset_visualization_commands();}, 
        [=](){return loggerViewUi->reset();}, 
        [=](bool set_sensitive){return commonroadViewUi->set_sensitive(set_sensitive);}, 
        argc, 
        argv);
    auto tabsViewUi = make_shared<TabsViewUI>(setupViewUi, vehicleManualControlUi, paramViewUi, timerViewUi, loggerViewUi, commonroadViewUi);
    auto mainWindow = make_shared<MainWindow>(tabsViewUi, monitoringUi, mapViewUi);

    //To create a window without Gtk complaining that no parent has been set, we need to pass the main window after mainWindow has been created
    //(Wherever we want to create windows)
    setupViewUi->set_main_window_callback(std::bind(&MainWindow::get_window, mainWindow));
    paramViewUi->set_main_window_callback(std::bind(&MainWindow::get_window, mainWindow));
    commonroadViewUi->set_main_window_callback(std::bind(&MainWindow::get_window, mainWindow));

    vehicleManualControl->set_callback([&](){vehicleManualControlUi->update();});

    return app->run(mainWindow->get_window());
}
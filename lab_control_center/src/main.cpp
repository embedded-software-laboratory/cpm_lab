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
#include "ui/lcc_errors/LCCErrorViewUI.hpp"
#include "ui/logger/LoggerViewUI.hpp"
#include "ui/setup/SetupViewUI.hpp"
#include "LogStorage.hpp"
#include "ParameterServer.hpp"
#include "ParameterStorage.hpp"
#include "RTTAggregator.hpp"
#include "TrajectoryCommand.hpp"
#include "ui/MainWindow.hpp"
#include "cpm/RTTTool.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "TimerTrigger.hpp"
#include "cpm/init.hpp"

#include "commonroad_classes/CommonRoadScenario.hpp"

#include "cpm/Writer.hpp"
#include "CommonroadDDSGoalState.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <functional>
#include <sstream>

//For exit handlers
#include <signal.h>
#include <stdlib.h>
#include <cstdlib>

using namespace std::placeholders;

//We need this to be a global variable, or else it cannot be used in the interrupt or exit handlers
std::shared_ptr<SetupViewUI> setupViewUi;

/**
 * \brief Function to deploy cloud discovery (to help participants discover each other)
 * \ingroup lcc
 */
void deploy_cloud_discovery() {
    std::string command = "tmux new-session -d -s \"rticlouddiscoveryservice\" \"rticlouddiscoveryservice -transport 25598\"";
    system(command.c_str());
}

/**
 * \brief Function to kill cloud discovery
 * \ingroup lcc
 */
void kill_cloud_discovery() {
    std::string command = "tmux kill-session -t \"rticlouddiscoveryservice\"";
    system(command.c_str());
}

/**
 * \brief Sometimes, a tmux session can stay open if the LCC crashed. 
 * To make sure that no session is left over / everything is "clean" 
 * when a new LCC gets started, kill all old sessions
 */
void kill_all_tmux_sessions() {
    std::string command = "tmux kill-server >>/dev/null 2>>/dev/null";
    system(command.c_str());
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
 * \brief Interrup handler of the LCC
 * \ingroup lcc
 */
void interrupt_handler(int s) {
    kill_cloud_discovery();

    //Kill remaining programs opened by setup view ui
    if (setupViewUi)
    {
        setupViewUi->on_lcc_close();
    }

    exit(1);
}

#pragma GCC diagnostic pop

/**
 * \brief Exit handler of the LCC
 * \ingroup lcc
 */
void exit_handler() {
    kill_cloud_discovery();

    //Kill remaining programs opened by setup view ui
    if (setupViewUi)
    {
        setupViewUi->on_lcc_close();
    }
}

/**
 * \brief Main function of the LCC.
 * Command line arguments:
 * 
 * --dds_domain
 * --logging_id
 * --dds_initial_peer
 * --simulated_time
 * --number_of_vehicles (default 20, set how many vehicles can max. be selected in the UI)
 * --config_file (default parameters.yaml)
 * \ingroup lcc
 */
int main(int argc, char *argv[])
{
    //Kill remaining tmux sessions
    kill_all_tmux_sessions();

    //Must be done first, s.t. no class using the logger produces an error
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("lab_control_center");
    cpm::RTTTool::Instance().activate("lab_control_center");

    //To receive logs as early as possible, and for Logging in main
    auto logStorage = make_shared<LogStorage>();

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
    }
    catch(const std::exception& e)
    {
        cpm::Logging::Instance().write(1, "Could not load initial commonroad scenario, error is: %s", e.what());
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
    auto timerViewUi = make_shared<TimerViewUI>(
        timerTrigger,
        obstacle_simulation_manager
    );
    auto loggerViewUi = make_shared<LoggerViewUI>(logStorage);
    auto vehicleManualControl = make_shared<VehicleManualControl>();
    auto vehicleAutomatedControl = make_shared<VehicleAutomatedControl>();
    auto trajectoryCommand = make_shared<TrajectoryCommand>();
    auto timeSeriesAggregator = make_shared<TimeSeriesAggregator>(255); //LISTEN FOR VEHICLE DATA UP TO ID 255 (for Commonroad Obstacles; is max. uint8_t value)
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
        [=](){return timeSeriesAggregator->get_vehicle_path_tracking_commands();},
        [=](){return obstacleAggregator->get_obstacle_data();}, 
        [=](){return visualizationCommandsAggregator->get_all_visualization_messages();}
    );
    auto rtt_aggregator = make_shared<RTTAggregator>();
    auto monitoringUi = make_shared<MonitoringUi>(
        deploy_functions, 
        [=](){return timeSeriesAggregator->get_vehicle_data();}, 
        [=](){return hlcReadyAggregator->get_hlc_ids_uint8_t();},
        [=](){return timeSeriesAggregator->get_vehicle_trajectory_commands();},
        [=](){return timeSeriesAggregator->reset_all_data();},
        [=](std::string id, uint64_t& c_best_rtt, uint64_t&  c_worst_rtt, uint64_t&  a_worst_rtt, double& missed_msgs)
            {
                return rtt_aggregator->get_rtt(id, c_best_rtt, c_worst_rtt, a_worst_rtt, missed_msgs);
            },
        [=](){return setupViewUi->kill_deployed_applications();}
    );
    auto vehicleManualControlUi = make_shared<VehicleManualControlUi>(vehicleManualControl);
    auto paramViewUi = make_shared<ParamViewUI>(storage, 5);
    auto commonroadViewUi = make_shared<CommonroadViewUI>(
        commonroad_scenario,
        obstacle_simulation_manager 
    );

    //Writer to send planning problems translated from commonroad to HLCs
    //As it is transient local, we need to reset the writer before each simulation start
    auto writer_planning_problems = std::make_shared<cpm::Writer<CommonroadDDSGoalState>>("commonroad_dds_goal_states", true, true, true);

    setupViewUi = make_shared<SetupViewUI>(
        deploy_functions,
        vehicleAutomatedControl, 
        hlcReadyAggregator, 
        [=](){return timeSeriesAggregator->get_vehicle_data();},
        [=](bool simulated_time, bool reset_timer){return timerViewUi->reset(simulated_time, reset_timer);}, 
        [&](){
            //Things to do when the simulation is started

            //Reset writer for planning problems (used down below), as it is transient local and we do not want to pollute the net with outdated data
            writer_planning_problems.reset();
            writer_planning_problems = std::make_shared<cpm::Writer<CommonroadDDSGoalState>>("commonroad_dds_goal_states", true, true, true);

            //Stop RTT measurement
            rtt_aggregator->stop_measurement();

            //Reset old UI elements (difference to kill: Also reset the Logs)
            //Kill timer in UI as well, as it should not show invalid information
            //Reset all relevant UI parts
            timeSeriesAggregator->reset_all_data();
            obstacleAggregator->reset_all_data();
            trajectoryCommand->stop_all();
            monitoringUi->notify_sim_start();
            visualizationCommandsAggregator->reset_visualization_commands();
            
            //We also reset the log file here - if you want to use it, make sure to rename it before you start a new simulation!
            loggerViewUi->reset();

            //Send commonroad planning problems to the HLCs (we use transient settings, so that the readers do not need to have joined)
            if(commonroad_scenario) commonroad_scenario->send_planning_problems(writer_planning_problems);

            //Start simulated obstacles - they will also wait for a start signal, so they are just activated to do so at this point
            obstacle_simulation_manager->stop(); //In case the preview has been used
            obstacle_simulation_manager->start();
            commonroadViewUi->reset_preview_label();

        }, 
        [=](){
            //Things to do when the simulation is stopped

            //Stop obstacle simulation
            obstacle_simulation_manager->stop();
            
            //Kill timer in UI as well, as it should not show invalid information
            //TODO: Reset Logs? They might be interesting even after the simulation was stopped, so that should be done separately/never (there's a log limit)/at start?
            //Reset all relevant UI parts
            timeSeriesAggregator->reset_all_data();
            obstacleAggregator->reset_all_data();
            trajectoryCommand->stop_all();
            monitoringUi->notify_sim_stop();
            visualizationCommandsAggregator->reset_visualization_commands();

            //Restart RTT measurement
            rtt_aggregator->restart_measurement();
        },
        [=](bool set_sensitive){return commonroadViewUi->set_sensitive(set_sensitive);}, 
        argc, 
        argv
    );
    monitoringUi->register_vehicle_to_hlc_mapping(
        [=](){return setupViewUi->get_vehicle_to_hlc_matching();}
    );
    monitoringUi->register_crash_checker(setupViewUi->get_crash_checker());
    timerViewUi->register_crash_checker(setupViewUi->get_crash_checker());
    auto lccErrorViewUi = make_shared<LCCErrorViewUI>();
    auto tabsViewUi = make_shared<TabsViewUI>(
        setupViewUi, 
        vehicleManualControlUi, 
        paramViewUi, 
        timerViewUi, 
        lccErrorViewUi,
        loggerViewUi, 
        commonroadViewUi);
    auto mainWindow = make_shared<MainWindow>(tabsViewUi, monitoringUi, mapViewUi);

    //To create a window without Gtk complaining that no parent has been set, we need to pass the main window after mainWindow has been created
    //(Wherever we want to create windows)
    setupViewUi->set_main_window_callback(std::bind(&MainWindow::get_window, mainWindow));
    paramViewUi->set_main_window_callback(std::bind(&MainWindow::get_window, mainWindow));
    commonroadViewUi->set_main_window_callback(std::bind(&MainWindow::get_window, mainWindow));

    vehicleManualControl->set_callback([&](){vehicleManualControlUi->update();});

    return app->run(mainWindow->get_window());
}
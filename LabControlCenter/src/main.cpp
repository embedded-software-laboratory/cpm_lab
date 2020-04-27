#include "defaults.hpp"
#include "stdio.h"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "TimeSeriesAggregator.hpp"
#include "HLCReadyAggregator.hpp"
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
    cpm::Logging::Instance().set_id("LabControlCenter");

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
    std::string filepath_2018 = "/home/cpm-lab/dev/software/LabControlCenter/test/C-USA_US101-30_1_T-1.xml";
    std::string filepath_2020 = "/home/cpm-lab/dev/software/LabControlCenter/test/documentation_XML_commonRoad_minimalExample_2020a.xml";
    std::string filepath_parked_vehicles = "/home/cpm-lab/dev/software/LabControlCenter/test/RUS_Bicycle-4_1_T-1.xml";
    std::string filepath_occupancy = "/home/cpm-lab/dev/software/LabControlCenter/test/DEU_Ffb-1_2_S-1.xml";
    auto commonroad_scenario = std::make_shared<CommonRoadScenario>(filepath_2018);
    commonroad_scenario->transform_coordinate_system(1.0, 0.0, 0.0);

    auto storage = make_shared<ParameterStorage>(config_file, 32);
    ParameterServer server(storage);
    storage->register_on_param_changed_callback(std::bind(&ParameterServer::resend_param_callback, &server, _1));

    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
    Glib::RefPtr<Gtk::CssProvider> cssProvider = Gtk::CssProvider::create();
    cssProvider->load_from_path("ui/style.css");
    Gtk::StyleContext::create()->add_provider_for_screen (Gdk::Display::get_default()->get_default_screen(),cssProvider,500);

    bool use_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    auto timerTrigger = make_shared<TimerTrigger>(use_simulated_time);
    auto timerViewUi = make_shared<TimerViewUI>(timerTrigger);
    auto logStorage = make_shared<LogStorage>();
    auto loggerViewUi = make_shared<LoggerViewUI>(logStorage);
    auto vehicleManualControl = make_shared<VehicleManualControl>();
    auto vehicleAutomatedControl = make_shared<VehicleAutomatedControl>();
    auto trajectoryCommand = make_shared<TrajectoryCommand>();
    auto timeSeriesAggregator = make_shared<TimeSeriesAggregator>();
    auto hlcReadyAggregator = make_shared<HLCReadyAggregator>();
    auto visualizationCommandsAggregator = make_shared<VisualizationCommandsAggregator>();
    auto mapViewUi = make_shared<MapViewUi>(
        trajectoryCommand, 
        commonroad_scenario,
        [=](){return timeSeriesAggregator->get_vehicle_data();},
        [=](){return timeSeriesAggregator->get_vehicle_trajectory_commands();},
        [=](){return visualizationCommandsAggregator->get_all_visualization_messages();}
    );
    auto monitoringUi = make_shared<MonitoringUi>(
        [=](){return timeSeriesAggregator->get_vehicle_data();}, 
        [=](){return hlcReadyAggregator->get_hlc_ids_string();},
        [=](){return timeSeriesAggregator->reset_all_data();}
    );
    auto vehicleManualControlUi = make_shared<VehicleManualControlUi>(vehicleManualControl);
    auto paramViewUi = make_shared<ParamViewUI>(storage, 5);
    auto setupViewUi = make_shared<SetupViewUI>(
        vehicleAutomatedControl, 
        [=](){return hlcReadyAggregator->get_hlc_ids_uint8_t();}, 
        [=](bool simulated_time){return timerViewUi->reset(simulated_time);}, 
        [=](){return timeSeriesAggregator->reset_all_data();}, 
        [=](){return trajectoryCommand->stop_all();}, 
        [=](){return monitoringUi->reset_vehicle_view();}, 
        [=](){return visualizationCommandsAggregator->reset_visualization_commands();}, 
        [=](){return loggerViewUi->reset();}, 
        argc, 
        argv);
    auto commonroadViewUi = make_shared<CommonroadViewUI>(commonroad_scenario, argc, argv);
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
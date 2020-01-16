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

void interrupt_handler(int s) {
    kill_cloud_discovery();
    exit(1);
}

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
    auto setupViewUi = make_shared<SetupViewUI>(timerViewUi, vehicleAutomatedControl, [=](){return hlcReadyAggregator->get_hlc_ids_uint8_t();}, argc, argv);
    auto tabsViewUi = make_shared<TabsViewUI>(setupViewUi, vehicleManualControlUi, paramViewUi, timerViewUi, loggerViewUi);
    auto mainWindow = make_shared<MainWindow>(tabsViewUi, monitoringUi, mapViewUi);


    vehicleManualControl->set_callback([&](){vehicleManualControlUi->update();});

    return app->run(mainWindow->get_window());
}
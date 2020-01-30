#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"
#include "VehicleAutomatedControl.hpp"
#include "cpm/CommandLineReader.hpp"
#include "ui/file_chooser/FileChooserUI.hpp"
#include "ui/timer/TimerViewUI.hpp"
#include "ui/setup/Deploy.hpp"
#include "ui/setup/VehicleToggle.hpp"
#include "ui/setup/UploadWindow.hpp"

#include "labcam/LabCamIface.hpp"

#include <atomic>
#include <array>
#include <cstdio> //For popen
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

/**
 * \brief This UI class is responsible for the Setup Tab in the LCC
 * It is used to 
 * 1) Set the variables required to start a local / distributed lab run, e.g. selection of a script + parameters, usage of the IPS in lab mode etc
 * 2) Run the selected script + vehicles (real and simulated possible) either on the current machine or on the lab's HLCs
 */
class SetupViewUI
{
private:
    //Builder and pointer to UI elements
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::ScrolledWindow* parent = nullptr;
    Gtk::Widget* setup_box = nullptr;

    //Script and script parameters
    Gtk::Entry* script_path = nullptr;
    Gtk::Entry* script_params = nullptr;
    Gtk::Button* button_choose_script = nullptr;
    
    //Vehicle selection
    Gtk::Button* button_select_none = nullptr;
    Gtk::Button* button_select_all_simulated = nullptr;
    Gtk::Button* button_select_all_real = nullptr;

    //Set timer (simulated or real time)
    Gtk::Switch* switch_simulated_time = nullptr;
    Gtk::Switch* switch_record_labcam = nullptr;

    //(De)Activate IPS
    Gtk::Switch* switch_lab_mode = nullptr;

    //(De)Activate remote deployment on HLCs (NUCs)
    Gtk::Switch* switch_deploy_remote = nullptr;

    //Start / stop simulation
    Gtk::Button* button_deploy = nullptr;
    Gtk::Button* button_kill = nullptr;

    //Vehicles - toggles in box to turn them on/off/simulated
    Gtk::FlowBox* vehicle_flowbox = nullptr;
    std::vector<std::shared_ptr<VehicleToggle>> vehicle_toggles;

    //Timer function - replace current timer in the whole system when user switches between simulated and real time
    std::shared_ptr<TimerViewUI> timer_ui;
    void switch_timer_set();

    // Interface to LabCam
    LabCamIface* labcam;

    //Class to send automated vehicle commands to a list of vehicles, like stop signals after kill has been called
    std::shared_ptr<VehicleAutomatedControl> vehicle_control;

    //Function to get a list of all currently online HLCs
    std::function<std::vector<uint8_t>()> get_hlc_ids;

    //Loading window while HLC scripts are being updated
    //Also: Upload threads and GUI thread (to keep upload work separate from GUI)
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::vector<std::thread> upload_threads; //threads that are responsible for uploading scripts to the HLCs
    std::shared_ptr<UploadWindow> upload_window; //window that shows an upload message
    void ui_dispatch(); //dispatcher callback for the UI thread
    void notify_upload_finished(); //notify function that gets called by the upload threads when they have finished their work
    void kill_all_threads(); //function to join all threads
    std::atomic_uint8_t thread_count; //thread counter, set before thread creation so that, if they finish before the next one is created, still threads are only joined after all upload threads that need to be created have finished their work
    size_t notify_count; //counter for notify_upload_finished; if it does not match thread_count after all threads have called it, print an error message (means that there was a setup mistake made at thread creation)
    std::mutex notify_callback_in_use; //the notify_upload_finished function should only be accessible by one thread at once, thus use this mutex
    std::atomic_bool upload_success; //Used by deploy and ui_dispatch in case the upload fails because no HLC was online or no vehicle was selected

    //IPS switch callback (-> lab mode)
    void switch_ips_set();

    //Overall deploy functions, to deploy / kill script + middleware + vehicle software locally /remotely
    void deploy_applications();
    void kill_deployed_applications();

    //Helper functions to get the currently selected vehicle IDs, IDs of real vehicles and IDs of simulated vehicles
    std::vector<unsigned int> get_vehicle_ids_active();
    std::vector<unsigned int> get_vehicle_ids_real();
    std::vector<unsigned int> get_vehicle_ids_simulated();

    //UI function - set sensitivity of the UI after pressing deploy (grey out fields in the Setup tab)
    void set_sensitive(bool is_sensitive);

    //Get parameters that were set in the command line (upon starting the LCC)
    bool cmd_simulated_time;
    unsigned int cmd_domain_id;
    std::string cmd_dds_initial_peer;

    //File chooser to select script(s) + location
    void open_file_explorer();
    void file_explorer_callback(std::string file_string, bool has_file);
    std::shared_ptr<FileChooserUI> file_chooser_window;

    //Vehicle button toggle callbacks, to set which vehicles are real / simulated / deactivated
    void select_all_vehicles_real();
    void select_all_vehicles_sim();
    void select_no_vehicles();

    //Class containing all functions that are relevant for deployment, local and remote
    std::shared_ptr<Deploy> deploy_functions;

public:
    /**
     * \brief Constructor
     * \param _timer_ui Allows to access the timer UI, to reset the timer on system restart
     * \param _vehicle_control Allows to send automated commands to the vehicles, like stopping them at their current position after simulation
     * \param _get_hlc_ids Get all IDs of currently active HLCs for correct remote deployment
     * \param argc Command line argument (from main())
     * \param argv Command line argument (from main())
     */
    SetupViewUI(std::shared_ptr<TimerViewUI> _timer_ui, std::shared_ptr<VehicleAutomatedControl> _vehicle_control, std::function<std::vector<uint8_t>()> _get_hlc_ids, unsigned int argc, char *argv[]);
    ~SetupViewUI();

    //Get the parent widget to put the view in a parent container
    Gtk::Widget* get_parent();
};
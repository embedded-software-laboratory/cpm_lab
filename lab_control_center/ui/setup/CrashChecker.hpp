#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

#include "ui/setup/Deploy.hpp"
#include "ui/setup/Upload.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/SimpleTimer.hpp"

#include "src/HLCReadyAggregator.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <vector>

/**
 * \brief This class is responsible for checking if background tasks that are supposed to be running are still running - else, a warning is displayed
 * It also reports wrong script/program locations set in the LCC for the main script
 * \ingroup lcc_ui
 */
class CrashChecker
{
private:
    //! Function to get the main window, as reference for the crash dialog window
    std::function<Gtk::Window&()> get_main_window;

    //! Access to deploy functions, used to check for crashes
    std::shared_ptr<Deploy> deploy_functions;

    //! Access to GUI thread to display warning dialog
    Glib::Dispatcher ui_dispatcher;
    /**
     * \brief Dispatcher callback for the UI thread
     */
    void ui_dispatch();
    
    //! Watcher thread that checks if the locally deployed programs still run - else, an error message is displayed
    std::thread thread_deploy_crash_check;
    //! To update newly / already crashed participants in update_crashed_participants and access them in ui_dispatch
    std::mutex crashed_mutex;
    //! List of participants whose crash has already been reported before
    std::unordered_set<std::string> already_crashed_participants;
    //! List of participants whose crash has not yet been reported
    std::vector<std::string> newly_crashed_participants;
    //! For the thread_deploy_crash_check thread, stop condition (stops if false)
    std::atomic_bool crash_check_running;
    //! Dialog window, displays crash messages / crashed participants to the user
    std::shared_ptr<Gtk::MessageDialog> crash_dialog;
    /**
     * \brief Function to kill the thread that checks for crashes
     */
    void kill_crash_check_thread();

    //Data structures for remote program crash check
    //! Mutex for accessing the hlc vectors
    std::mutex hlc_id_mutex;
    //! List of currently running remote HLCs where the desired scripts still run as well.
    std::vector<uint8_t> running_remote_hlcs;
    //! List of crashed remote HLCs / HLCs where scripts crashed, used by check_if_crashed. If a participant / the scripts running on it come back online again, it is removed.
    std::vector<uint8_t> crashed_remote_hlcs;

    //! Access to remote program check, to see if script and middleware are still running on remote HLCs (used for e.g. crashed_remote_hlcs vector)
    std::shared_ptr<HLCReadyAggregator> hlc_ready_aggregator;
    //! To check if an upload was finished - remote crash checks cannot be performed before that
    std::shared_ptr<Upload> upload_manager;
    //! To remember the last time when the upload to the HLCs was not finished - use a timeout afterwards, with remote_waiting_time, to wait some more before checking for crashes remotely
    uint64_t upload_success_time = 0;
    //! Start reporting remote crashes 5 seconds after the upload finished (messages seem to "stutter" / are not correct directly after the upload)
    const uint64_t remote_waiting_time = 5e9;

    //Further helper functions
    /**
     * \brief Used by the crash check thread, checks for program crashes on HLCs / HLC crashes after an upload of the script was performed
     */
    std::vector<std::string> check_for_remote_crashes();
    /**
     * \brief Used by crash check thread, updates newly_crashed_participants / already_... after a crash check, 
     * triggers the UI if a new crash was detected to create a crash dialog
     */
    void update_crashed_participants(std::vector<std::string> crashed_participants);

public:
    /**
     * \brief Constructor
     * \param _deploy_functions Needed to find out if local programs are still running
     * \param _hlc_ready_aggregator Required to find out if remote programs are still running
     * \param _upload_manager Required to check if an upload was finished, for remote checking
     */
    CrashChecker(
        std::shared_ptr<Deploy> _deploy_functions,
        std::shared_ptr<HLCReadyAggregator> _hlc_ready_aggregator,
        std::shared_ptr<Upload> _upload_manager
    );

    /**
     * \brief Destructor, stops the crash check thread
     */
    ~CrashChecker();

    /**
     * \brief Start checking if the deployed applications are still running
     * \param script_used If a script + middleware was deployed (else only check for crashes of IPS etc) 
     * \param use_middleware_without_hlc Self explanatory, ignored in case of distributed / remote deployment
     * \param remote_hlc_ids If deployed remotely: IDs of HLCs on which the software was deployed
     * \param has_local_hlc True if local HLCs are used on top of remote ones in distributed mode
     * \param remote_deploy_toggled True if Deploy Remote / Distributed is enabled in SetupViewUI
     * \param lab_mode_on Whether the IPS etc. should be running
     * \param labcam_toggled If true, the labcam program should be running too
     */
    void start_checking(bool script_used, bool use_middleware_without_hlc, std::vector<uint8_t> remote_hlc_ids, bool has_local_hlc, bool remote_deploy_toggled, bool lab_mode_on, bool labcam_toggled);

    /**
     * \brief Stop checking for crashes
     */
    void stop_checking();

    /**
     * \brief This function can be used to check if the program deployed on an HLC with the given ID crashed (remote)
     * \param hlc_id ID of the HLC
     * \return True if the checker is running and the program crashed, else false
     */
    bool check_if_crashed(uint8_t hlc_id);

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);
};

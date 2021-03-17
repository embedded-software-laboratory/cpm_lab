#include "CrashChecker.hpp"

/**
 * \file CrashChecker.cpp
 * \ingroup lcc_ui
 */

CrashChecker::CrashChecker(
    std::shared_ptr<Deploy> _deploy_functions,
    std::shared_ptr<HLCReadyAggregator> _hlc_ready_aggregator,
    std::shared_ptr<Upload> _upload_manager
) :
    deploy_functions(_deploy_functions),
    hlc_ready_aggregator(_hlc_ready_aggregator),
    upload_manager(_upload_manager)
{
    //Create connection to UI thread
    ui_dispatcher.connect(sigc::mem_fun(*this, &CrashChecker::ui_dispatch));

    crash_check_running.store(false);
}

CrashChecker::~CrashChecker()
{
    kill_crash_check_thread();
}

void CrashChecker::kill_crash_check_thread()
{
    crash_check_running.store(false);
    if (thread_deploy_crash_check.joinable())
    {
        thread_deploy_crash_check.join();
    }
}

void CrashChecker::ui_dispatch()
{
    //Create dialog window that informs the user about a crash of one of the programs they started (besides vehicles, this can be seen in the UI already)
    std::unique_lock<std::mutex> lock_crashes(crashed_mutex);
    if (newly_crashed_participants.size() > 0)
    {
        std::stringstream crash_report;
        crash_report << "Newly crashed programs:\n";
        for (auto& program : newly_crashed_participants)
        {
            crash_report << "\t" << program << "\n";
        }
        crash_report << "\nPreviously crashed programs:\n";
        for (auto& program : already_crashed_participants)
        {
            crash_report << "\t" << program << "\n";
        }

        //Remember previous crashes, clear for new crashed - do this both in UI and check thread, as they interoperate non-deterministically
        for (auto& participant : newly_crashed_participants)
        {
            already_crashed_participants.insert(participant);
        }
        newly_crashed_participants.clear();

        //Close window if it is still open
        if (crash_dialog)
        {
            crash_dialog->close();
            crash_dialog.reset();
        }

        //Required for the creation of the dialog window -> see set_main_window_callback
        assert(get_main_window);

        //Create new window
        crash_dialog = std::make_shared<Gtk::MessageDialog>(
            get_main_window(),
            crash_report.str(),
            false,
            Gtk::MessageType::MESSAGE_INFO,
            Gtk::ButtonsType::BUTTONS_CLOSE,
            false
        );
    
        //Connect new window with parent, show window
        crash_dialog->set_transient_for(get_main_window());
        crash_dialog->property_destroy_with_parent().set_value(true);
        crash_dialog->show();

        //Callback for closing
        crash_dialog->signal_response().connect(
            [this] (auto response)
            {
                if (response == Gtk::ResponseType::RESPONSE_CLOSE)
                {
                    crash_dialog->close();
                }
            }
        );
    }
    lock_crashes.unlock();
}

std::vector<std::string> CrashChecker::check_for_remote_crashes()
{
    //running_remote_hlcs should maybe also be updated if a NUC went offline/crashed, but reporting this twice would also not be too bad 
    //(Report would then be: Programs XY have crashed, NUC crashed -> probably even better this way)
    std::vector<std::string> crashed_programs;

    //Only start checking if an upload was performed; remember starting time when upload was finished to add some extra delay before checking
    if (! upload_manager->upload_finished())
    {
        upload_success_time = cpm::get_time_ns();
        return crashed_programs;
    }

    //Additional waiting time (make sure that remote_waiting time is longer than the periodic call of this check, else this is skipped)
    if (cpm::get_time_ns() - upload_success_time < remote_waiting_time)
    {
        return crashed_programs;
    }

    //Check for answers for all IDs, we made sure that some time has passed before missing answers are being reported 
    //(from testing experience: in the beginning, the connection is not stable enough to assume that the other party is offline,
    //and there are tests for offline NUCs already)
    std::lock_guard<std::mutex> lock(hlc_id_mutex);
    for (auto id_iterator = running_remote_hlcs.begin(); id_iterator != running_remote_hlcs.end();)
    {
        auto id_string = std::to_string(static_cast<int>(*id_iterator));

        auto script_running = hlc_ready_aggregator->script_running_on(*id_iterator);
        auto middleware_running = hlc_ready_aggregator->middleware_running_on(*id_iterator);

        if (!script_running)
        {
            cpm::Logging::Instance().write("Script crashed on NUC %s (remote)", id_string.c_str());

            std::stringstream report_stream;
            report_stream << "Script at HLC " << id_string;
            crashed_programs.push_back(report_stream.str());
        }

        if (!middleware_running)
        {
            cpm::Logging::Instance().write("Middleware crashed on NUC %s (remote)", id_string.c_str());

            std::stringstream report_stream;
            report_stream << "Middleware at HLC " << id_string;
            crashed_programs.push_back(report_stream.str());
        }

        if(!script_running || !middleware_running)
        {
            //Erase entry s.t. the message does not appear again
            //TODO: Currently, this means that, if only one of them crashed, the other crash will no longer be reported. Is that acceptable?
            crashed_remote_hlcs.push_back(*id_iterator);
            id_iterator = running_remote_hlcs.erase(id_iterator);
        }
        else
        {
            ++id_iterator;
        }
    }

    //Sometimes, crash detection fails (remote), because of timeouts or wrong messages in the beginning
    //Manual re-starts of programs on NUCs are also possible
    //Thus: Re-integrate an ID if it is detected to be online again
    for (auto id_iterator = crashed_remote_hlcs.begin(); id_iterator != crashed_remote_hlcs.end();)
    {
        auto script_running = hlc_ready_aggregator->script_running_on(*id_iterator);
        auto middleware_running = hlc_ready_aggregator->middleware_running_on(*id_iterator);

        if(script_running && middleware_running)
        {
            //Put entry back in entries that are checked
            running_remote_hlcs.push_back(*id_iterator);
            id_iterator = crashed_remote_hlcs.erase(id_iterator);
        }
        else
        {
            ++id_iterator;
        }
    }

    return crashed_programs;
}

void CrashChecker::start_checking(bool script_used, bool use_middleware_without_hlc, std::vector<uint8_t> remote_hlc_ids, bool has_local_hlc, bool remote_deploy_toggled, bool lab_mode_on, bool labcam_toggled)
{
    kill_crash_check_thread();

    //Copy hlc IDs s.t. they can be removed for remote deploy
    running_remote_hlcs = remote_hlc_ids;
    crashed_remote_hlcs.clear();

    //First-time update of upload success time, in case upload is finished before its success could be checked
    //Is used to calculate time-diff before checking for remote crashes
    upload_success_time = cpm::get_time_ns();

    //Deploy crash check thread
    crash_check_running.store(true);
    thread_deploy_crash_check = std::thread(
        [this, script_used, use_middleware_without_hlc, remote_hlc_ids, has_local_hlc, remote_deploy_toggled, lab_mode_on, labcam_toggled] () {
            //Give programs time to actually start
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            //Warn once in case no script was used
            bool script_used_warned = false;

            while(crash_check_running.load())
            {
                auto crashed_participants_local = deploy_functions->check_for_crashes(script_used, remote_deploy_toggled, has_local_hlc, lab_mode_on, labcam_toggled);

                std::vector<std::string> crashed_participants_remote;
                if (script_used)
                    crashed_participants_remote = check_for_remote_crashes();
                

                std::vector<std::string> crashed_participants;
                crashed_participants.reserve(crashed_participants_local.size() + crashed_participants_remote.size());
                crashed_participants.insert(crashed_participants.end(), crashed_participants_local.begin(), crashed_participants_local.end());
                crashed_participants.insert(crashed_participants.end(), crashed_participants_remote.begin(), crashed_participants_remote.end());

                //Display msg once if no script was started due to path settings or that was not intended for local deployment
                if (!script_used_warned && !script_used && !(use_middleware_without_hlc && (remote_hlc_ids.size() == 0)))
                {
                    crashed_participants.push_back("HLC/Middleware (invalid path set by user)");
                    script_used_warned = true;
                }

                update_crashed_participants(crashed_participants);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    );
}

void CrashChecker::update_crashed_participants(std::vector<std::string> crashed_participants)
{
    if (crashed_participants.size() > 0)
    {                    
        std::lock_guard<std::mutex> lock(crashed_mutex);

        //Remember previous crashes, clear for new crashed
        for (auto& participant : newly_crashed_participants)
        {
            already_crashed_participants.insert(participant);
        }
        newly_crashed_participants.clear();

        //Store all new crashes so that they can be shown in the UI separately (New and old crashes)
        bool new_crash_detected = false;
        for (auto& participant : crashed_participants)
        {
            if (already_crashed_participants.find(participant) == already_crashed_participants.end())
            {
                newly_crashed_participants.push_back(participant);
                new_crash_detected = true;
            }
        }

        //If a new crash was detected, notify the UI thread. It will check the size of newly_crashed participants and create a dialog if is greater than zero
        if (new_crash_detected)
        {
            //Log the new crash
            std::stringstream program_stream;
            program_stream << "New: ";
            for (auto& entry : newly_crashed_participants)
            {
                program_stream << entry << " | ";
            }

            if (already_crashed_participants.size() > 0)
            {
                program_stream << " - Previous: ";
                for (auto& entry : already_crashed_participants)
                {
                    program_stream << entry << " | ";
                }
            }
            
            cpm::Logging::Instance().write("The following programs crashed during simulation: %s", program_stream.str().c_str());

            ui_dispatcher.emit();
        }
    }
}

void CrashChecker::stop_checking()
{
    kill_crash_check_thread();
    running_remote_hlcs.clear();
    crashed_remote_hlcs.clear();
    already_crashed_participants.clear();
    newly_crashed_participants.clear();
    upload_success_time = 0;
}

bool CrashChecker::check_if_crashed(uint8_t hlc_id)
{
    std::lock_guard<std::mutex> lock(hlc_id_mutex);
    return (std::find(crashed_remote_hlcs.begin(), crashed_remote_hlcs.end(), hlc_id) != crashed_remote_hlcs.end());
}

void CrashChecker::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}

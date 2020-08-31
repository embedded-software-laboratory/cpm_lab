#include "CrashChecker.hpp"

CrashChecker::CrashChecker(
    std::shared_ptr<Deploy> _deploy_functions
) :
    deploy_functions(_deploy_functions)
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
            true
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

void CrashChecker::start_checking(bool deploy_remote_toggled, bool lab_mode_on, bool labcam_toggled)
{
    //Deploy crash check thread
    crash_check_running.store(true);
    thread_deploy_crash_check = std::thread(
        [this, deploy_remote_toggled, lab_mode_on, labcam_toggled] () {
            //Give programs time to actually start
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            while(crash_check_running.load())
            {
                auto crashed_participants = deploy_functions->check_for_crashes(deploy_remote_toggled, lab_mode_on, labcam_toggled);

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

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    );
}

void CrashChecker::stop_checking()
{
    kill_crash_check_thread();
}

void CrashChecker::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}
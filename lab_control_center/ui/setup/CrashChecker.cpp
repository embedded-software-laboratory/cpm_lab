#include "CrashChecker.hpp"

CrashChecker::CrashChecker(
    std::shared_ptr<Deploy> _deploy_functions
) :
    deploy_functions(_deploy_functions),
    program_check_topic(cpm::get_topic<RemoteProgramCheck>(cpm::ParticipantSingleton::Instance(), "remote_program_check")),
    program_check_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), program_check_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepAll())),
    program_check_reader(
        [&](dds::sub::LoanedSamples<RemoteProgramCheck>& samples){
            std::lock_guard<std::mutex> lock(map_mutex);
            auto current_count = check_msg_count.load();
            for(auto sample : samples)
            {
                if(sample.info().valid())
                {
                    //Only print answers
                    if ((sample.data().is_answer()))
                    {
                        auto id = sample.data().source_id();

                        //Store new values, use count to find out if the values are new
                        auto sample_count = sample.data().count();
                        if (sample_count == current_count)
                        {
                            script_running[id] = sample.data().script_running();
                            middleware_running[id] = sample.data().middleware_running();
                        }
                    }
                }
            }
        },
        cpm::ParticipantSingleton::Instance(),
        program_check_topic,
        true,
        false
    )
{
    //Create connection to UI thread
    ui_dispatcher.connect(sigc::mem_fun(*this, &CrashChecker::ui_dispatch));

    crash_check_running.store(false);

    check_msg_count.store(0);
    is_first_run.store(false);
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

    if (remote_crash_check)
    {
        remote_crash_check->stop();
    }
    remote_crash_check.reset();

    //Reset data structures as well - only msgs with count 0 could be received / stored afterwards, and these will be reset 
    //again before a new check starts, so this should not be problematic
    check_msg_count.store(0);
    std::lock_guard<std::mutex> lock(map_mutex);
    script_running.clear();
    middleware_running.clear();
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

void CrashChecker::send_remote_check_msg()
{
    if (check_msg_count < 255)
    {
        ++check_msg_count;
    }
    else
    {
        check_msg_count = 0;
    }
    

    //Send program check request
    RemoteProgramCheck check_msg;
    check_msg.source_id("lcc_program_check");
    check_msg.is_answer(false);
    check_msg.count(check_msg_count);
    program_check_writer.write(check_msg);
}

std::vector<std::string> CrashChecker::check_for_remote_crashes(std::vector<uint8_t>& remote_hlc_ids)
{
    //remote_hlc_ids should maybe also be updated if a NUC went offline/crashed, but reporting this twice would also not be too bad 
    //(Report would then be: Programs XY have crashed, NUC crashed -> probably even better this way)
    std::vector<std::string> crashed_programs;

    //Check for answers for all IDs, we made sure that some time has passed before missing answers are being reported 
    //(from testing experience: in the beginning, the connection is not stable enough to assume that the other party is offline,
    //and there are tests for offline NUCs already)
    for (auto id_iterator = remote_hlc_ids.begin(); id_iterator != remote_hlc_ids.end();)
    {
        auto id_string = std::to_string(static_cast<int>(*id_iterator));

        std::lock_guard<std::mutex> lock(map_mutex);
        if (script_running.find(id_string) != script_running.end())
        {
            //Entry exists in both maps, check if the script and the middleware are still running
            if (middleware_running.at(id_string) && script_running.at(id_string))
            {
                ++id_iterator;
            }
            else
            {
                if (! script_running.at(id_string))
                {
                    cpm::Logging::Instance().write("Script crashed on NUC %s (remote)", id_string.c_str());

                    std::stringstream report_stream;
                    report_stream << "Script at HLC " << id_string;
                    crashed_programs.push_back(report_stream.str());
                }

                if (! middleware_running.at(id_string))
                {
                    cpm::Logging::Instance().write("Middleware crashed on NUC %s (remote)", id_string.c_str());

                    std::stringstream report_stream;
                    report_stream << "Middleware at HLC " << id_string;
                    crashed_programs.push_back(report_stream.str());
                }

                //Erase entry s.t. the message does not appear again
                //TODO: Currently, this means that, if only one of them crashed, the other crash will no longer be reported. Is that acceptable?
                id_iterator = remote_hlc_ids.erase(id_iterator);
            }

            //Delete entry s.t. we do not assume next time that we received an answer
            script_running.erase(id_string);
            middleware_running.erase(id_string);
        }
        else
        {
            cpm::Logging::Instance().write("NUC %s did not respond to program online check", id_string.c_str());
            //TODO: Maybe actually remove IDs of offline NUCs to prevent confusion regarding this msg
        }
    }

    return crashed_programs;
}

void CrashChecker::start_checking(std::vector<uint8_t> remote_hlc_ids, bool lab_mode_on, bool labcam_toggled)
{
    kill_crash_check_thread();

    //Deploy crash check threads 
    //We need two threads because for the remote checks, due to traffic & response times, checking for running programs takes much longer
    //Although the waiting times are similar within the while loop, check_for_remote_crashes has much longer waiting times

    crash_check_running.store(true);
    thread_deploy_crash_check = std::thread(
        [this, remote_hlc_ids, lab_mode_on, labcam_toggled] () {
            //Give programs time to actually start
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            while(crash_check_running.load())
            {
                auto crashed_participants = deploy_functions->check_for_crashes((remote_hlc_ids.size() > 0), lab_mode_on, labcam_toggled);

                update_crashed_participants(crashed_participants);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    );

    //We need to use a simple timer for remote checks because the waiting times for answers need to be long
    //aborting the simulation would else take too long due to thread joining
    hlc_ids_copy = remote_hlc_ids;
    remote_crash_check = std::make_shared<cpm::SimpleTimer>("lcc_remote_check", 2000, false, false);
    remote_crash_check->start_async(
        [this, remote_hlc_ids] (uint64_t t_now)
        {
            auto first_run = is_first_run.load();

            if (first_run)
            {
                //TODO: Somehow wait until the upload is finished before performing checks
                //Alternative: Only start checks after any first message has arrived
            }

            //Check for answers to the message sent in the last run
            if(! first_run)
            {
                auto crashed_participants = check_for_remote_crashes(hlc_ids_copy);

                update_crashed_participants(crashed_participants);
            }

            send_remote_check_msg();
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
}

void CrashChecker::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}
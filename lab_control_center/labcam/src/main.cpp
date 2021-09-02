/**
 * \file main.cpp
 * 
 * \brief Starts labcam recording. The command line parameters path and file_name can be used to
 *        specify the folder and filename of the resulting recording. As soon as a SIGTERM or
 *        SIGHUP signal is recognized, the recording is stopped.
 * 
 * \ingroup lcc_labcam
 */

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <thread>
#include "labcam/LabCamIface.hpp"
#include "cpm/CommandLineReader.hpp"

//! Variable containing the interface to the actual labcam.
static LabCamIface labcam;


sig_atomic_t volatile is_stopped = false;

/**
 * \brief Handler, which stops labcam recording as soon as the program is to be closed externally.
 * 
 * In the current workflow, the labcam is started in an own tmux session. As soon as the tmux session
 * is closed by another process, the labcam should stop recording. In order to allow a proper finish,
 * this handler is necessary to let the camera stop properly.
 * 
 * \ingroup lcc_labcam
 */
void stop_signal_handler(int){
    std::cout << "Stop-signal received." << std::endl;
    labcam.stopRecording();
    is_stopped = true;
};

int main(int argc, char *argv[])
{
    // Connect signal handler for termination signal
    signal(SIGTERM, stop_signal_handler);
    signal(SIGHUP, stop_signal_handler);

    // Read command line input
    // If no path is given, the default location is software/lab_control_center/build/labcam (due to "." and creation of tmux session)
    std::string path = cpm::cmd_parameter_string("path", ".", argc, argv);
    std::string file_name = cpm::cmd_parameter_string("file_name", "awesome_recording", argc, argv);

    // Start recording by using the given input parameters
    labcam.startRecording(path, file_name);

    std::chrono::milliseconds dt_sleep(500);
    while(~is_stopped) std::this_thread::sleep_for(dt_sleep);
    return 0;
}

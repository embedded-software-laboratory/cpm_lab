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

/**
 * \file main.cpp
 * 
 * \brief Starts labcam recording. The command line parameters path and file_name can be used to
 *        specify the folder and filename of the resulting recording. As soon as a SIGTERM or
 *        SIGHUP signal is recognized, the recording is stopped.
 * 
 * \ingroup lcc_labcam
 */

#include <stdlib.h>
#include <iostream>
#include <csignal>
#include <string>
#include "labcam/LabCamIface.hpp"
#include "cpm/CommandLineReader.hpp"

//! Variable containing the interface to the actual labcam.
static LabCamIface labcam;

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

    while(1);
}

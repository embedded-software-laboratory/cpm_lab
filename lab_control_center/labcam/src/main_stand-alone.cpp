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
 * \file main_stand-alone.cpp
 * 
 * \brief Starts labcam recording in an interactive way for testing purposes. The labcam
 *        can be stopped by pressing enter and restarted once before the program terminates.
 * 
 * \ingroup lcc_labcam
 */

#include <stdlib.h>
#include <iostream>
#include <string>
#include "labcam/LabCam.hpp"

//Suppress warning for unused parameter of main
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

int main(int argc, char *argv[])
{
    LabCam labcam;

    labcam.startRecording(".", "awesome_recording1");

    std::cout << "." << std::endl;
    std::cin.get();

    std::cout << "stopping lab cam" << std::endl;
    labcam.stopRecording();
    std::cout << "Press enter to start recording again" << std::endl;

    
    std::cin.get();
    labcam.startRecording(".", "awesome_recording2");


    std::cin.get();
    labcam.stopRecording();

    while(1);
}

#pragma GCC diagnostic pop

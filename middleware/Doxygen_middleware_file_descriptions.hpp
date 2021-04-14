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

//File descriptions for middleware files

/**
* \page mw_files Middleware Files
* \subpage mw_build <br>
* \subpage mw_run <br>
* \subpage qos_loc <br>
* \ingroup middleware_files
*/

/**
 * \page mw_build build.bash
 * \brief Build script for the middleware
 * 
 * Also creates a package to download for the NUC/HLC when it boots to get the latest version of the middleware.
 */

/**
 * \page mw_run run.bash
 * \brief Run script for the middleware.
 * 
 * Works given a vehicle ID (or multiple IDs, comma-separated) and a parameter that determines if simulated time should be used (true / false).
 */

/**
* \page qos_loc QOS_LOCAL_COMMUNICATION.xml.template
* \brief Contains QoS settings
* 
* QoS settings for the DDS participant used for the local-only communication between middleware and HLC. 
* See https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
*/
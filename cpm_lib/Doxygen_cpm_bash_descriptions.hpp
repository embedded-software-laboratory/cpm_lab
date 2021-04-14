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

//This file includes descriptions for the cpm bash scripts and for the test folder

/**
 * \defgroup cpmlib_further_files Further CPM Lib Files
 * \brief Additional files for the CPM Lib, e.g. Bash scripts
 * \ingroup cpmlib
 */

/**
* \page cpm_files CPM Files
* \subpage cpm_build <br>
* \subpage cpm_build_arm <br>
* \subpage cpm_rtigen <br>
* \subpage cpm_rtigen_matlab <br>
* \subpage cpm_test <br>
* \ingroup cpmlib_further_files
*/

/**
 * \page cpm_build build.bash
 * \brief x86 build file for the cpm library. Also creates C++ and Matlab files from the .idl DDS type definitions, if the according
 * folders do not already exist. If they do, please remove the folders beforehand if the files should be re-generated.
 * Runs created tests.
 * Also creates one of the packages for the HLC/NUC s.t. it also has access to the library and generated files.
 */

/**
 * \page cpm_build_arm build_arm.bash
 * \brief ARM build file for the cpm library
 */

/**
 * \page cpm_rtigen rtigen.bash
 * \brief Creates C++ files from the .idl type definitions for DDS messages
 */

/**
 * \page cpm_rtigen_matlab rtigen_matlab.m
 * \brief Creates Matlab files from the .idl type definitions for DDS messages
 */

/**
 * \page cpm_test cpm_lib/test
 * \brief Includes test files 
 */
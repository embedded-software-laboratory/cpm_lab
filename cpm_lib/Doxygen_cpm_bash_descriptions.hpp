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
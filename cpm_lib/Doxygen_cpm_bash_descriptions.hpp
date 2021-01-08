//This file includes descriptions for the cpm bash scripts and for the test folder

/**
 * \file build.bash
 * \brief x86 build file for the cpm library. Also creates C++ and Matlab files from the .idl DDS type definitions, if the according
 * folders do not already exist. If they do, please remove the folders beforehand if the files should be re-generated.
 * Runs created tests.
 * Also creates one of the packages for the HLC/NUC s.t. it also has access to the library and generated files.
 * \ingroup cpmlib
 */

/**
 * \file build_arm.bash
 * \brief ARM build file for the cpm library
 * \ingroup cpmlib
 */

/**
 * \file rtigen.bash
 * \brief Creates C++ files from the .idl type definitions for DDS messages
 * \ingroup cpmlib
 */

/**
 * \file rtigen_matlab.m
 * \brief Creates Matlab files from the .idl type definitions for DDS messages
 * \ingroup cpmlib
 */

/**
 * \dir cpm_lib/test
 * \brief Includes test files 
 * \ingroup cpmlib
 */
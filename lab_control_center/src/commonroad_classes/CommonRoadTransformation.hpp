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

#pragma once

#include "cpm/Logging.hpp"

#include <atomic>
#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

//For storing transformation profile
#include <yaml-cpp/yaml.h>
#include <experimental/filesystem> //Used instead of std::filesystem, because some compilers still seem to be outdated

/**
 * \brief Used to remember transformations for files during execution, if saved, and to save / load those in between executions using YAML
 */
class CommonRoadTransformation
{
private:
    //Config file that stores the applied transformation for each file name
    const std::string transformation_file_location = "./commonroad_profiles.yaml";

    //Name of current file
    std::string current_file_name = "";

    //Loaded profile
    std::mutex yaml_profile_mutex;
    YAML::Node yaml_transform_profile; //'Working' profile that is immediately changed with each change made by the user ("cache" which might not be used)
    YAML::Node old_yaml_transform_profile; //Remembers state after last save of current setting, s.t. reset to those when loading again is possible without reloading the file
    const std::string time_scale_key = "time_scale";
    const std::string scale_key = "min_lane_width";
    const std::string translate_x_key = "translate_x";
    const std::string translate_y_key = "translate_y";
    const std::string rotation_key = "z_rotation";
    std::atomic_bool profile_applied;

    /**
     * \brief Helper function to set default values (0.0) for undefined transformation keys (when loading or adding values, for missing values)
     * Only call w. locked transform mutex & if the profile exists!
     */ 
    void set_defaults_for_undefined(YAML::Node& profile);

    /**
     * \brief Floor the given value to 0 if smaller min_value
     */
    double floor_small_values(double val);

public:
    /**
     * \brief Constructor, loads the YAML file or creates it if it does not yet exist
     */
    CommonRoadTransformation();

    /**
     * \brief Set the name of the scenario so that its transformation profile can be saved properly
     * \param scenario_path Path of the scenario, including its name (.../...xml); only the name is used
     */
    void set_scenario_name(std::string scenario_path);

    const double min_value = 1.0e-6;
    /**
     * \brief Load, if exists, the stored transformation for the current file; values smaller than min_value are set to zero (to avoid values like 6.553e-310)
     * \param time_scale Output, 0.0 if empty, analog to time step size
     * \param min_lane_width Output, 0.0 if empty, analog to lane width
     * \param translate_x Output, 0.0 if empty, translation of coordinates in x direction
     * \param translate_y Output, 0.0 if empty, translation of coordinates in y direction
     * \param rotation Ouput, 0.0 if empty, rotation of the coordinates around the origin, counter-clockwise
     */   
    void load_transformation_from_profile(double& time_scale, double& min_lane_width, double& translate_x, double& translate_y, double& rotation);

    /**
     * \brief Store the transform profile that stores previously used transformations for commonroad scenarios
     */
    void store_transform_profile();

    /**
     * \brief Function that adds new transformation values for the currently selected file to the transform profile; values smaller than min_value are set to zero (to avoid values like 6.553e-310)
     * Relative to stored change if profile was loaded before, else overwritten
     * \param time_scale Ignored if <= 0.0, analog to time step size
     * \param min_lane_width Ignored if <= 0.0, analog to lane width
     * \param translate_x Translation of coordinates in x direction
     * \param translate_y Translation of coordinates in y direction
     * \param rotation Rotation of the coordinates around the origin, counter-clockwise
     */
    void add_change_to_transform_profile(double time_scale, double min_lane_width, double translate_x, double translate_y, double rotation);

    /**
     * \brief Reset transform profile for the currently selected files
     */
    void reset_current_transform_profile();
};
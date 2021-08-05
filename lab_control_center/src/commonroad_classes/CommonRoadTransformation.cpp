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

#include "commonroad_classes/CommonRoadTransformation.hpp"

/**
 * \file CommonRoadTransformation.cpp
 * \ingroup lcc_commonroad
 */

CommonRoadTransformation::CommonRoadTransformation()
{
    //Make sure that the file exists, else create a file first
    std::experimental::filesystem::path filepath = transformation_file_location;
    bool file_exists = std::experimental::filesystem::exists(filepath);

    if (!file_exists)
    {
        //Create file
        std::ofstream yaml_file(transformation_file_location);
        yaml_file.close();
    }

    //Now load the profile
    std::lock_guard<std::mutex> lock(yaml_profile_mutex);

    yaml_transform_profile = YAML::LoadFile(transformation_file_location);

    //Make sure that the file has not been edited / malformed manually - in that case, clear its content
    if (! yaml_transform_profile.IsNull())
    {
        if (! yaml_transform_profile.IsMap())
        {
            std::cout << "NOTE: Resetting YAML transform profile, is malformed" << std::endl;
            std::ofstream yaml_file;
            yaml_file.open(transformation_file_location, std::ofstream::out | std::ofstream::trunc);
            yaml_file.close();

            yaml_transform_profile = YAML::LoadFile(transformation_file_location);
        }
    }
    
    old_yaml_transform_profile = YAML::Clone(yaml_transform_profile);

    //If a profile was applied, we need to consider this when saving (because changes are then 'incremental' instead of 'overwriting')
    profile_applied.store(false);
}

void CommonRoadTransformation::set_scenario_name(std::string scenario_path)
{
    std::lock_guard<std::mutex> lock(yaml_profile_mutex);

    auto filename_reduced = scenario_path;
    auto last_slash = filename_reduced.find_last_of('/');
    if (last_slash != std::string::npos)
    {
        filename_reduced = filename_reduced.substr(last_slash + 1);
    }
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), ':'), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), '\\'), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), '\''), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), '"'), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), '['), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), ']'), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), '{'), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), '}'), filename_reduced.end());
    filename_reduced.erase(std::remove(filename_reduced.begin(), filename_reduced.end(), ' '), filename_reduced.end());

    current_file_name = filename_reduced;
    profile_applied.store(false);

    //Reset yaml profile to last saved state / get rid of changes that were not supposed to be stored
    yaml_transform_profile = YAML::Clone(old_yaml_transform_profile);
}

double CommonRoadTransformation::floor_small_values(double val)
{
    if (val < min_value && val > -(min_value)) return 0.0;

    return val;
}

void CommonRoadTransformation::load_transformation_from_profile(double& time_scale, double& scale, double& translate_x, double& translate_y, double& rotation)
{
    std::lock_guard<std::mutex> lock(yaml_profile_mutex);

    if (current_file_name != "" && old_yaml_transform_profile[current_file_name])
    {
        //Load profile
        auto profile = old_yaml_transform_profile[current_file_name];

        //Make sure that values exist
        if (! (profile[time_scale_key] && profile[scale_key] && profile[translate_x_key] && profile[translate_y_key] && profile[rotation_key]))
        {
            cpm::Logging::Instance().write(1, "Commonroad profile not properly defined for %s in %s, setting default values...", current_file_name.c_str(), transformation_file_location.c_str());
            set_defaults_for_undefined(profile);
        }
    
        //Load values from profile
        time_scale = floor_small_values(profile[time_scale_key].as<double>());
        scale = floor_small_values(profile[scale_key].as<double>());
        translate_x = floor_small_values(profile[translate_x_key].as<double>());
        translate_y = floor_small_values(profile[translate_y_key].as<double>());
        rotation = floor_small_values(profile[rotation_key].as<double>());

        //Reset changes that have not been stored
        yaml_transform_profile = YAML::Clone(old_yaml_transform_profile);
    }
    else
    {
        //Default values
        time_scale = 0.0;
        scale = 0.0;
        translate_x = 0.0;
        translate_y = 0.0;
        rotation = 0.0;
    }
    
    profile_applied.store(true);
}

void CommonRoadTransformation::store_transform_profile()
{
    std::lock_guard<std::mutex> lock(yaml_profile_mutex);

    //Store changed profile
    std::ofstream yaml_file(transformation_file_location, std::ofstream::out | std::ofstream::trunc);
    yaml_file << yaml_transform_profile;
    yaml_file.close();

    //Now also keep the change for loading from the newly set profile (copy)
    old_yaml_transform_profile = YAML::Clone(yaml_transform_profile);
}

void CommonRoadTransformation::add_change_to_transform_profile(double time_scale, double scale, double translate_x, double translate_y, double rotation)
{
    std::lock_guard<std::mutex> lock(yaml_profile_mutex);

    //TODO: Rotation values are currently being ignored, because it is harder to store them properly (they are non-linear when applied)
    rotation = 0.0;

    if (current_file_name != "")
    {
        //Load profile
        auto profile = yaml_transform_profile[current_file_name];

        double translate_x_old = 0.0;
        double translate_y_old = 0.0;
        double rotation_old = 0.0;
        double scale_old = 0.0;

        //Only regard old values if they have been loaded before
        if (profile && profile_applied.load())
        {
            //Make sure that values exist
            if (! (profile[time_scale_key] && profile[scale_key] && profile[translate_x_key] && profile[translate_y_key] && profile[rotation_key]))
            {
                cpm::Logging::Instance().write(1, "Commonroad profile not properly defined for %s in %s, setting default values...", current_file_name.c_str(), transformation_file_location.c_str());
                set_defaults_for_undefined(profile);
            }

            //Load old values from profile
            translate_x_old = floor_small_values(profile[translate_x_key].as<double>());
            translate_y_old = floor_small_values(profile[translate_y_key].as<double>());
            rotation_old = floor_small_values(profile[rotation_key].as<double>());
            scale_old = floor_small_values(profile[scale_key].as<double>());
        }

        if (scale_old > 0.0)
        {
            //When the YAML-file is loaded, we first apply transformation before applying scale
            //Transform values that were added after a scale would thus be applied wrongly, thus we need to adjust them before storing them
            translate_x /= scale_old;
            translate_y /= scale_old;
        }

        //Store values
        double new_scale = scale * scale_old;
        if ( new_scale > 0 ) profile[scale_key] = floor_small_values(new_scale);
        else if ( scale > 0 ) profile[scale_key] = floor_small_values(scale);

        if ( time_scale > 0 ) profile[time_scale_key] = floor_small_values(time_scale);
        profile[translate_x_key] = floor_small_values(translate_x_old + translate_x);
        profile[translate_y_key] = floor_small_values(translate_y_old + translate_y);
        profile[rotation_key] = floor_small_values(std::fmod((rotation_old + rotation), 2.0 * M_PI)); //Modulo 2*Pi for rotation value

        //Now we are in a situation where, if the profile was applied before or not, we now use the same profile as the one stored in the profile node
        //As changes are additive from here on, we need to reuse old values
        profile_applied.store(true);
    }
}

void CommonRoadTransformation::reset_current_transform_profile()
{
    //First reset the current profile
    {
        std::lock_guard<std::mutex> lock(yaml_profile_mutex);

        if (current_file_name != "")
        {
            //Load profile
            auto profile = yaml_transform_profile[current_file_name];

            if (profile)
            {
                yaml_transform_profile.remove(current_file_name);
            }

            old_yaml_transform_profile = YAML::Clone(yaml_transform_profile);
        }
    }

    //Then store that change
    store_transform_profile();
}

void CommonRoadTransformation::set_defaults_for_undefined(YAML::Node& profile)
{
    
    if (! (profile[time_scale_key]))
    {
        profile[time_scale_key] = 0.0;
    }

    if (! (profile[scale_key]))
    {
        profile[scale_key] = 0.0;
    }

    if (! (profile[translate_x_key]))
    {
        profile[translate_x_key] = 0.0;
    }

    if (! (profile[translate_y_key]))
    {
        profile[translate_y_key] = 0.0;
    }

    if (! (profile[rotation_key]))
    {
        profile[rotation_key] = 0.0;
    }
}
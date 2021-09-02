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
 * \ingroup lcc_commonroad
 */
class CommonRoadTransformation
{
private:
    //! Config file that stores the applied transformation for each file name
    const std::string transformation_file_location = "./commonroad_profiles.yaml";

    //! Name of current file
    std::string current_file_name = "";

    //Loaded profile
    //! Mutex to change the data for the transform profiles
    std::mutex yaml_profile_mutex;
    //! 'Working' profile that is immediately changed with each change made by the user ("cache" which might not be used)
    YAML::Node yaml_transform_profile;
    //! Remembers state after last save of current setting, s.t. reset to those when loading again is possible without reloading the file
    YAML::Node old_yaml_transform_profile;
    //! Key for the time scale value in a profile
    const std::string time_scale_key = "time_scale";
    //! Key for the scale scale value in a profile
    const std::string scale_key = "scale";
    //! Key for the translate_x value in a profile
    const std::string translate_x_key = "translate_x";
    //! Key for the translate_y value in a profile
    const std::string translate_y_key = "translate_y";
    //! Key for the rotation value in a profile
    const std::string rotation_key = "z_rotation";
    //! If a profile was applied before saving the new transformations, we need to consider this when saving (because changes are then 'incremental' instead of 'overwriting')
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

    //! To floor small values; smaller values are not allowed
    const double min_value = 1.0e-6;
    
    /**
     * \brief Load, if exists, the stored transformation for the current file; values smaller than min_value are set to zero (to avoid values like 6.553e-310)
     * \param time_scale Output, 0.0 if empty, analog to time step size
     * \param scale Output, 0.0 if empty
     * \param translate_x Output, 0.0 if empty, translation of coordinates in x direction
     * \param translate_y Output, 0.0 if empty, translation of coordinates in y direction
     * \param rotation Currently not supported! Ouput, 0.0 if empty, rotation of the coordinates around the origin, counter-clockwise
     */   
    void load_transformation_from_profile(double& time_scale, double& scale, double& translate_x, double& translate_y, double& rotation);

    /**
     * \brief Store the transform profile that stores previously used transformations for commonroad scenarios
     */
    void store_transform_profile();

    /**
     * \brief Function that adds new transformation values for the currently selected file to the transform profile; values smaller than min_value are set to zero (to avoid values like 6.553e-310)
     * Relative to stored change if profile was loaded before, else overwritten
     * \param time_scale Ignored if <= 0.0, analog to time step size
     * \param scale Ignored if <= 0.0
     * \param translate_x Translation of coordinates in x direction
     * \param translate_y Translation of coordinates in y direction
     * \param rotation Currently not supported! Rotation of the coordinates around the origin, counter-clockwise
     */
    void add_change_to_transform_profile(double time_scale, double scale, double translate_x, double translate_y, double rotation);

    /**
     * \brief Reset transform profile for the currently selected files
     */
    void reset_current_transform_profile();
};
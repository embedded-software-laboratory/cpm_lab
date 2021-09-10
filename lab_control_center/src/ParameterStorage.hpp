#pragma once

#include "ParameterWithDescription.hpp"

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <cassert>
#include <iostream>
#include <fstream>
#include <exception>

/**
 * \class ParameterStorage
 * \brief This class can be used in combination with YAML files and the parameter server. 
 * Stores parameter values set by the user (in the LCC's UI) or directly in the YAML file
 * Do not confuse this class with the class in the cpm lib! That class is used on the client's side.
 * \ingroup lcc
 */
class ParameterStorage {
public:
    /**
     * \brief Constructor
     * \param _filename Filename of the YAML file which stores saved parameter data for the program
     * \param precision Floating point precision to be used within YAML, lower values allow i.e. for better storage efficiency
     */
    ParameterStorage(std::string _filename, int precision);

    //Delete move and copy op
    ParameterStorage(ParameterStorage const&) = delete;
    ParameterStorage(ParameterStorage&&) = delete; 
    ParameterStorage& operator=(ParameterStorage const&) = delete;
    ParameterStorage& operator=(ParameterStorage &&) = delete;

    /**
     * \brief used to register a callback that is called whenever the value of a parameter changes
     * \param _on_param_changed_callback the callback function
     */
    void register_on_param_changed_callback(std::function<void(std::string)> _on_param_changed_callback);

    /**
     * \brief Load YAML file into memory, use mutex
     * \param _filename Change the current filename and use the given parameter for that, thus switch to the new file
     */
    void loadFile(std::string _filename);
    /**
     * \brief Load YAML file into memory, use mutex and filename stored in this class from the constructor
     */
    void loadFile();

    /**
     * \brief Store current configuration in YAML file
     * \param _filename Change the current filename and use the given parameter for that, thus switch to the new file
     */
    void storeFile(std::string _filename);
    /**
     * \brief Store current configuration in YAML file, use filename stored in this class from the constructor
     */
    void storeFile();

    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_bool(std::string name, bool value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_uint64_t(std::string name, uint64_t value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_int(std::string name, int32_t value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_double(std::string name, double value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_string(std::string name, std::string value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_string(std::string name, const char* value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_ints(std::string name, std::vector<int32_t> value, std::string info = "");
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     * \param info description of the parameter's purpose
     */
    void set_parameter_doubles(std::string name, std::vector<double> value, std::string info = "");

    /**
     * \brief Abstracted function used by all set_parameter_... functions to actually store the parameter.
     * Also used by the parameter ui class
     * \param name Name of the parameter
     * \param param Parameter values
     */
    void set_parameter(std::string name, ParameterWithDescription param);

    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_bool(std::string name, bool& value);
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_uint64_t(std::string name, uint64_t& value);
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_int(std::string name, int32_t& value);
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_double(std::string name, double& value);
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_string(std::string name, std::string& value);
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_ints(std::string name, std::vector<int32_t>& value);
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_parameter_doubles(std::string name, std::vector<double>& value);

    /**
     * \brief Abstracted function used by all get_parameter_... functions to actually get the parameter.
     * Also used by the parameter ui class
     * \param name Name of the parameter
     * \param param Parameter values
     */
    bool get_parameter(std::string name, ParameterWithDescription& param);

    /**
     * \brief Delete a parameter
     * \param name The parameter's name
     */
    void delete_parameter(std::string name);

    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_bool();
    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_uint64_t();
    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_int();
    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_double();
    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_string();
    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_ints();
    /**
     * \brief Returns a list all existing parameter names of that type
     */
    std::vector<std::string> list_doubles();

    /**
     * \brief Returns all stored parameters
     */
    std::vector<ParameterWithDescription> get_all_parameters();

    /**
     * \brief Returns the set floating point precision for the YAML storage
     */
    int get_precision();
private:
    /**
     * \brief Internal function used by the list_... functions to list the name of all parameters of some type
     * \param type The parameter type for which to list all parameter names
     */
    std::vector<std::string> list_names(ParameterType type);

    //! Float / double precision for YAML, default value is 32
    int PRECISION = 32;

    //! Name / location of the YAML file in which parameters can be stored / from which they can be loaded
    std::string filename;

    //! Parameter storage during the LCC's execution
    std::map<std::string, ParameterWithDescription> param_storage;

    //! Mutex for access to param_storage
    std::mutex param_storage_mutex;

    /**
     * \brief Callback that gets called whenever a parameter changes, to send its new value to other participants in the network.
     * Is set with register_on_param_changed_callback.
     */
    std::function<void(std::string)> on_param_changed_callback;
};
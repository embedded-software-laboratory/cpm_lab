#pragma once

/**
 * \class ParameterStorage
 * \brief This class can be used in combination with yaml files and the parameter server
 */

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

class ParameterStorage {
public:
    /**
     * \brief Load YAML file into memory
     */
    void loadFile();
    /**
     * \brief Store current configuration in YAML file
     */
    void storeFile();
    /**
     * \brief Set the value of a parameter
     * \param name name of the parameter
     * \param value new value of the parameter
     */
    void set_typename(std::string name, bool value); //TODO
    /**
     * \brief Get the value of a parameter
     * \param name name of the parameter
     * \param value value of the parameter
     * \return true if the parameter exists
     */
    bool get_typename(std::string name, bool& value); //TODO
    /**
     * \brief List all existing parameters of that type
     */
    std::vector<std::string> list_typename(); //TODO
private:

};
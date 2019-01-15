#pragma once

/**
 * \class Parameter.hpp
 * \brief This interface can be used to receive/request constant definitions of different types
 * Parameters can be set during runtime using the parameter server. All values set are sent to all participants that are listening to the server. These parameter values are made up of a parameter name, a parameter type and the value of the parameter. This interface can be used to receive or request the current definition of a paramter (which can be of type bool, int, double, string or an array of ints or doubles). 
 */

#include <string>
#include <vector>

namespace cpm {
    /**
     * \brief retrieve the value of a bool parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    bool parameter_bool(std::string parameter_name);
    /**
     * \brief retrieve the value of an int parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    int32_t parameter_int(std::string parameter_name);
    /**
     * \brief retrieve the value of a double parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    double parameter_double(std::string parameter_name);
    /**
     * \brief retrieve the value of a string parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    std::string parameter_string(std::string parameter_name);
    /**
     * \brief retrieve the value of a vector-of-ints parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    std::vector<int32_t> parameter_ints(std::string parameter_name);
    /**
     * \brief retrieve the value of a vector-of-doubles parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    std::vector<double> parameter_doubles(std::string parameter_name);
    /**
     * \brief retrieve the value of a vector-of-strings parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    std::vector<std::string> parameter_strings(std::string parameter_name);
}
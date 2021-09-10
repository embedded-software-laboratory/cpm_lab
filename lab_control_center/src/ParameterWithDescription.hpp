#pragma once

#include "cpm/dds/Parameter.hpp"
#include <string>
#include <sstream>
#include <iomanip> 
#include <iostream>

/**
 * \struct ParameterWithDescription
 * \brief To be used with ParameterStorage, to save parameter information as well as their description 
 * \ingroup lcc
 */ 
struct ParameterWithDescription {
    //! DDS representation of parameter data, meaning name and value, without its description
    Parameter parameter_data;
    //! Additional optional description of the parameter, to explain its use, which is shown in the UI
    std::string parameter_description;

    /**
     * \brief Helper function to obtain values from the struct as a string
     * \param param The param to obtain values from
     * \param name Return value: The obtained name of the parameter as string
     * \param type Return value: The obtained type of the parameter as string
     * \param value Return value: The obtained value of the parameter as string, with set precision for doubles
     * \param info Return value: The obtained info of the parameter as string
     * \param precision Precision with which to obtain doubles
     */
    static void parameter_to_string(ParameterWithDescription& param, std::string &name, std::string &type, std::string &value, std::string &info, int precision) {
        name = param.parameter_data.name();
        
        if (param.parameter_data.type() == ParameterType::Bool) {
            type = "Bool";
            std::stringstream value_stream;
            value_stream << param.parameter_data.value_bool();
            value = value_stream.str();
        }
        else if (param.parameter_data.type() == ParameterType::Int32) {
            type = "Integer";
            std::stringstream value_stream;
            value_stream << param.parameter_data.values_int32().at(0);
            value = value_stream.str();
        }
        else if (param.parameter_data.type() == ParameterType::UInt64) {
            type = "UInt64 (/Timestamp)";
            std::stringstream value_stream;
            value_stream << param.parameter_data.value_uint64_t();
            value = value_stream.str();
        }
        else if (param.parameter_data.type() == ParameterType::Double) {
            type = "Double";
            std::stringstream value_stream;
            value_stream << std::fixed << std::setprecision(precision) << param.parameter_data.values_double().at(0);
            value = value_stream.str();
        }
        else if (param.parameter_data.type() == ParameterType::String) {
            type = "String";
            value = param.parameter_data.value_string();
        }
        else if (param.parameter_data.type() == ParameterType::Vector_Int32) {
            type = "Integer List";
            std::stringstream value_stream;
            value_stream << param.parameter_data.values_int32();
            value = value_stream.str();
        }
        else if (param.parameter_data.type() == ParameterType::Vector_Double) {
            type = "Double List";
            std::stringstream value_stream;
            value_stream << std::fixed << std::setprecision(precision) << param.parameter_data.values_double();
            value = value_stream.str();
        }

        info = param.parameter_description;
    }
};
#pragma once

#include "cpm/dds/Parameter.hpp"
#include <string>
#include <sstream>
#include <iomanip> 
#include <iostream>

/**
 * \brief To be used with ParameterStorage, to save parameter information as well as their description 
 */ 

struct ParameterWithDescription {
    Parameter parameter_data;
    std::string parameter_description;

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
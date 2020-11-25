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
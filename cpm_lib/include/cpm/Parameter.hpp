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

/**
 * \file Parameter.hpp
 * \brief This interface can be used to receive/request parameter definitions of different types
 * Parameters can be set during runtime using the parameter 
 * server. All values set are sent to all participants that 
 * are listening to the server. These parameter values are 
 * made up of a parameter name, a parameter type and the value 
 * of the parameter. This interface can be used to  
 * request the current definition of a paramter. 
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
     * \brief retrieve the value of a uint64_t parameter
     * \param parameter_name the name of the parameter
     * \return the value of the parameter with the given name
     */
    uint64_t parameter_uint64_t(std::string parameter_name);
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

}

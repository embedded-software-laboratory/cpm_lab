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

#include <string>
#include <sstream>
#include <vector>

/**
 * \brief Used to read a single command line argument from argv
 */

namespace cpm {
    /**
     * \brief Read a boolean command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    bool cmd_parameter_bool(std::string name, bool default_value, int argc, char *argv[]);

    /**
     * \brief Read an integer command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    int cmd_parameter_int(std::string name, int default_value, int argc, char *argv[]);

    /**
     * \brief Read an integer command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    uint64_t cmd_parameter_uint64_t(std::string name, uint64_t default_value, int argc, char *argv[]);

    /**
     * \brief Read a double command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    double cmd_parameter_double(std::string name, double default_value, int argc, char *argv[]);

    /**
     * \brief Read a std::string command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    std::string cmd_parameter_string(std::string name, std::string default_value, int argc, char *argv[]);

    /**
     * \brief Read an integer command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    std::vector<int> cmd_parameter_ints(std::string name, std::vector<int> default_value, int argc, char *argv[]);

    /**
     * \brief Read a double command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    std::vector<double> cmd_parameter_doubles(std::string name, std::vector<double> default_value, int argc, char *argv[]);
    /**
     * \brief Read an float array command line argument from argv (form: --name=value,value), use a default value if it does not exist
     */
    std::vector<float> cmd_parameter_floats(std::string name, std::vector<float> default_value, int argc, char *argv[]);

}
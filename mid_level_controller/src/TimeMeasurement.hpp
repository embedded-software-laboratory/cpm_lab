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
#include <memory>
#include <map>
#include <ctime>
#include "cpm/get_time_ns.hpp"


// see https://stackoverflow.com/questions/1008019/c-singleton-design-pattern


/**
 * \class MeasurementData
 * Just for internal use.
 */
class MeasurementData {
    public:
        uint64_t start_time = 0;
        uint64_t end_time = 0;
        clockid_t clockid;

        MeasurementData(clockid_t clockid);
};




/**
 * \class TimeMeasurement.hpp
 * This class comprises functions with which time measurements between specific points in the code can be made.
 * One instance saves all measurements which are currently done (Singleton).
 */

class TimeMeasurement {
    public:
        static TimeMeasurement& Instance();

        // Neccessary deletion of functions when working with singleton pattern
        TimeMeasurement(TimeMeasurement const&) = delete;
        void operator=(TimeMeasurement const&)  = delete;

        /**
         * This function starts a measurement. If there is already a measurment with this name the old data will be overriden.
         * \param name  The name of the measurement. All data for the measurment is saved under this name.
         * \param clockid The clockid which will be used for this measurement.
         */
        void start(std::string name, clockid_t clockid);

        /**
         * Same as start above but uses default clockid.
         */
        void start(std::string name);

        /**
         * This function stops a measurement.
         * \param name  The name of the measurement which is to be stopped.
         * \return      The result of the measurement.
         */
        uint64_t stop(std::string name);

        /**
         * Returns all measurements in string format.
         */
        std::string get_str();

        /**
         * Configure the clockid which is used as default when calling start() without specifying the clockid.
         */
        void set_default_clockid(clockid_t clockid);


    private:
        TimeMeasurement(){}
        static TimeMeasurement& instance;

        clockid_t default_clockid = CLOCK_MONOTONIC;
        std::map<std::string, MeasurementData> measurements;

};


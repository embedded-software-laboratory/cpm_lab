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


#include <string>
#include <memory>
#include <map>
#include "cpm/Timer.hpp"


// see https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

/**
 * \class TimeMeasurement.hpp
 * This class comprises functions with which time measurements between specific points in the code can be made.
 * One instance saves all measurements which are currently done (Singleton).
 */

class TimeMeasurement {
    public:
        static TimeMeasurement& Instance();

        /**
         * This function starts a measurement.
         * \param name  The name of the measurement. All data for the measurment is saved under this name.
         * \param timer The timer which will be used for this measurement.
         */
        void start(std::string name, std::shared_ptr<cpm::Timer> timer);

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

    
    private:
        TimeMeasurement();
        static TimeMeasurement& instance;

        std::map<std::string, MeasurementData> measurements;

};

/**
 * \class MeasurementData
 * Just for internal use.
 */
class MeasurementData {
    public:
        uint64_t start_time;
        uint64_t end_time;
        std::shared_ptr<cpm::Timer> timer;
};
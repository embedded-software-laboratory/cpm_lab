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


namespace cpm
{

    /**
     * \class MeasurementData
     * Just for internal use.
     */
    class MeasurementData {
        public:
            //! TODO
            uint64_t start_time = 0;
            //! TODO
            uint64_t end_time = 0;
            //! TODO
            clockid_t clockid;

            /**
             * \brief TODO
             * \param clockid TODO
             */
            MeasurementData(clockid_t clockid);
    };




    /**
     * \class TimeMeasurement.hpp
     * This class comprises functions with which time measurements between specific points in the code can be made.
     * One instance saves all measurements which are currently done (Singleton).
     */

    class TimeMeasurement {
        public:
            /**
             * \brief Provides access to the Singleton / creates it
             */
            static TimeMeasurement& Instance();

            // Neccessary deletion of functions when working with singleton pattern
            TimeMeasurement(TimeMeasurement const&) = delete;
            void operator=(TimeMeasurement const&)  = delete;

            /**
             * \brief This function starts a measurement. If there is already a measurment with this name the old data will be overriden.
             * \param name  The name of the measurement. All data for the measurment is saved under this name.
             * \param clockid The clockid which will be used for this measurement.
             */
            void start(std::string name, clockid_t clockid);

            /**
             * \brief Same as start above but uses default clockid.
             * \param name The name of the measurement. All data for the measurment is saved under this name.
             */
            void start(std::string name);

            /**
             * \brief This function stops a measurement.
             * \param name  The name of the measurement which is to be stopped.
             * \return      The result of the measurement.
             */
            uint64_t stop(std::string name);

            /**
             * \brief Returns all measurements in string format.
             */
            std::string get_str();

            /**
             * \brief Configure the clockid which is used as default when calling start() without specifying the clockid.
             * \param clockid TODO
             */
            void set_default_clockid(clockid_t clockid);

            /**
             * \brief Returns true iff a measurement with the given name exists (active or finished)
             * \param name Measurement name
             */
            bool exists(std::string name);


        private:
            /**
             * \brief Private constructor (due to Singleton)
             */
            TimeMeasurement(){}
            //! Singleton instance
            static TimeMeasurement& instance;

            //! TODO
            clockid_t default_clockid = CLOCK_MONOTONIC;
            //! TODO
            std::map<std::string, MeasurementData> measurements;

    };

} // namespace cpm

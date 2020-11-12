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


#include "TimeMeasurement.hpp"
#include "cpm/Logging.hpp"
#include "cpm/get_time_ns.hpp"


TimeMeasurement& TimeMeasurement::Instance(){
    static TimeMeasurement instance; // see Meyer's Singleton
    return instance;
}


void TimeMeasurement::start(std::string name, clockid_t clockid){
    std::map<std::string, MeasurementData>::iterator it = measurements.find(name);
    if (it != measurements.end()){
        // Already an existing element. Override.
        measurements.erase(it);
    }

    // Create new data object and insert it in the map
    MeasurementData data(clockid);
    measurements.insert(std::pair<std::string, MeasurementData>(name, data));
}


void TimeMeasurement::start(std::string name){
    this->start(name, this->default_clockid);
}


uint64_t TimeMeasurement::stop(std::string name){
    std::map<std::string, MeasurementData>::iterator it = measurements.find(name);
    if (it == measurements.end()){
        // Element not existing. Log warning and return 0.
        cpm::Logging::Instance().write(
            2,
            "Warning: Tried to stop a non-existing time measurement by name %s",
            name.c_str()
        );
        return 0;
    }

    MeasurementData& data = it->second;
    data.end_time = cpm::get_time_ns(data.clockid);

    return data.end_time - data.start_time;
}


std::string TimeMeasurement::get_str(){
    std::string res = "Time Measurement";

    for (auto const& it : measurements){
        res += " | " + it.first + ":";
        if (it.second.end_time == 0){
            // Measurement not finished
            res += std::to_string(cpm::get_time_ns(it.second.clockid) - it.second.start_time) + "(nf)";
        }
        else {
            res += std::to_string(it.second.end_time - it.second.start_time);
        }
    }

    return res;
}


void TimeMeasurement::set_default_clockid(clockid_t clockid){
    this->default_clockid = clockid;
}



MeasurementData::MeasurementData(clockid_t clockid){
    // Init here or within : ... ?
    this->clockid = clockid;
    this->start_time = cpm::get_time_ns(clockid);
}
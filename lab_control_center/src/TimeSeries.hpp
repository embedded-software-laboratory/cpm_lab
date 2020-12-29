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
#include "defaults.hpp"
#include "VehicleCommandTrajectory.hpp"
#include "cpm/get_time_ns.hpp"

/**
 * \brief Data class for storing values & (receive) times to get latest / newest data etc
 * \ingroup lcc
 */
template<typename T>
class _TimeSeries
{
    vector<function<void(_TimeSeries&, uint64_t time, T value)>> new_sample_callbacks;

    vector<uint64_t> times;
    vector<T> values;

    const string name;
    const string format;
    const string unit;

    mutable std::mutex m_mutex;

public:

    _TimeSeries(string _name, string _format, string _unit);
    void push_sample(uint64_t time, T value);
    string format_value(double value);
    T get_latest_value() const;
    uint64_t get_latest_time() const;
    bool has_new_data(double dt) const;
    bool has_data() const;
    string get_name() const {return name;}
    string get_unit() const {return unit;}
    vector<T> get_last_n_values(size_t n) const;

};

using TimeSeries = _TimeSeries<double>;
using TimeSeries_TrajectoryPoint = _TimeSeries<TrajectoryPoint>;
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

#include "TimeSeries.hpp"

template<typename T>
_TimeSeries<T>::_TimeSeries(string _name, string _format, string _unit)
:name(_name)
,format(_format)
,unit(_unit)
{
    times.push_back(0);
    values.push_back(T());
}


template<typename T>
void _TimeSeries<T>::push_sample(uint64_t time, T value) 
{
    // Lock scope
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        times.push_back(time);
        values.push_back(value);   
    }

    for(auto callback : new_sample_callbacks)
    {
        if(callback)
        {
            callback(*this, time, value);
        }
    }
}


template<typename T>
string _TimeSeries<T>::format_value(double value) 
{
    return string_format(format, value);
}

template<typename T>
T _TimeSeries<T>::get_latest_value() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return values.back();
}


template<typename T>
bool _TimeSeries<T>::has_new_data(double dt) const 
{
    const uint64_t age = double(clock_gettime_nanoseconds() - get_latest_time());
    return age/1e9 < dt;
}

template<typename T>
bool _TimeSeries<T>::has_data() const 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return ! (values.empty());
}

template<typename T>
uint64_t _TimeSeries<T>::get_latest_time() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return times.back();
}

template<typename T>
vector<T> _TimeSeries<T>::get_last_n_values(size_t n) const 
{
    if(values.size() <= n) return values;

    return vector<T>(values.end()-n, values.end());
}

template class _TimeSeries<double>;
template class _TimeSeries<TrajectoryPoint>;
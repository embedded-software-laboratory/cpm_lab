#pragma once
#include "defaults.hpp"

class TimeSeries
{
    vector<function<void(TimeSeries&, uint64_t time, double value)>> new_sample_callbacks;

public:

    vector<uint64_t> times;
    vector<double> values;

    string name;
    string format;
    string unit;

    TimeSeries(string _name, string _format, string _unit);

    void push_sample(uint64_t time, double value);

    void add_new_sample_callback(function<void(TimeSeries&, uint64_t time, double value)> callback)
    { new_sample_callbacks.push_back(callback); }
    
};
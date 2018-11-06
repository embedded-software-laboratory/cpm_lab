#pragma once
#include "defaults.hpp"

class TimeSeries
{

    vector<uint64_t> times;
    vector<double> values;

    string name;
    string format;
    string unit;

public:
    TimeSeries();

    void push_sample(uint64_t time, double value);
    
};
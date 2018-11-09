#pragma once
#include "defaults.hpp"

class TimeSeries
{
    vector<function<void(TimeSeries&, uint64_t time, double value)>> new_sample_callbacks;

    vector<uint64_t> times;
    vector<double> values;

    const string name;
    const string format;
    const string unit;

    std::mutex m_mutex;

public:

    TimeSeries(string _name, string _format, string _unit);
    void push_sample(uint64_t time, double value);
    string format_value(double value);
    double get_latest_value();
    void add_new_sample_callback(function<void(TimeSeries&, uint64_t time, double value)> callback);
    string get_name() const {return name;}
    string get_unit() const {return unit;}
};
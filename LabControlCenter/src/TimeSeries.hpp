#pragma once
#include "defaults.hpp"
#include "VehicleCommandTrajectory.hpp"

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
    string get_name() const {return name;}
    string get_unit() const {return unit;}
    vector<T> get_last_n_values(size_t n) const;

};

using TimeSeries = _TimeSeries<double>;
using TimeSeries_TrajectoryPoint = _TimeSeries<TrajectoryPoint>;
#include "TimeSeries.hpp"

TimeSeries::TimeSeries(string _name, string _format, string _unit)
:name(_name)
,format(_format)
,unit(_unit)
{
    times.push_back(0);
    values.push_back(0);
}


void TimeSeries::push_sample(uint64_t time, double value) 
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

void TimeSeries::add_new_sample_callback(function<void(TimeSeries&, uint64_t time, double value)> callback)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    new_sample_callbacks.push_back(callback); 
}


string TimeSeries::format_value(double value) 
{
    return string_format(format, value);
}

double TimeSeries::get_latest_value() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return values.back();
}


bool TimeSeries::has_new_data(double dt) const 
{
    const uint64_t age = double(clock_gettime_nanoseconds() - get_latest_time());
    return age/1e9 < dt;
}

uint64_t TimeSeries::get_latest_time() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return times.back();
}

vector<double> TimeSeries::get_last_n_values(size_t n) const 
{
    if(values.size() <= n) return values;

    return vector<double>(values.end()-n, values.end());
}
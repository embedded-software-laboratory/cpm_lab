#include "TimeSeries.hpp"

template<typename T>
_TimeSeries<T>::_TimeSeries(string _name, string _format, string _unit)
:name(_name)
,format(_format)
,unit(_unit)
{
    times.push_back(0);
    values.push_back(0);
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
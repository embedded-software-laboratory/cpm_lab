#include "TimeSeries.hpp"

TimeSeries::TimeSeries(string _name, string _format, string _unit)
:name(_name)
,format(_format)
,unit(_unit)
{
    
}


void TimeSeries::push_sample(uint64_t time, double value) 
{
    times.push_back(time);
    values.push_back(value);

    for(auto callback : new_sample_callbacks)
    {
        if(callback)
        {
            callback(*this, time, value);
        }
    }
}
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "cpm/SimpleTimer.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include "cpm/get_topic.hpp"

/**
 * \file SimpleTimer.cpp
 * \ingroup cpmlib
 */
namespace cpm {

    SimpleTimer::SimpleTimer(
        std::string _node_id, 
        uint64_t _period_milliseconds, 
        bool _wait_for_start,
        bool _react_to_stop_signal,
        uint64_t stop_signal
    )
    {
        internal_timer = std::make_shared<cpm::TimerFD>(_node_id, fifty_ms, 0, _wait_for_start, stop_signal);
        counter_max = static_cast<uint64_t>(std::ceil(_period_milliseconds / 50));
        internal_timer_counter = counter_max; //Send first message at start

        if (! _react_to_stop_signal)
        {
            m_stop_callback = [] () {};
        }
    }

    void SimpleTimer::simple_timer_callback(uint64_t t_now)
    {
        ++internal_timer_counter;
        if (internal_timer_counter >= counter_max)
        {
            internal_timer_counter = 0;

            if (m_update_callback)
            {
                m_update_callback(t_now);
            }
            else
            {
                cpm::Logging::Instance().write(
                    2, 
                    "%s", 
                    "Callback function for simple timer is undefined!"
                );
            }
        }
    }

    using namespace std::placeholders;

    void SimpleTimer::start(std::function<void(uint64_t t_now)> update_callback)
    {
        m_update_callback = update_callback;

        if (m_stop_callback)
        {
            internal_timer->start(std::bind(&SimpleTimer::simple_timer_callback, this, _1), m_stop_callback);
        }
        else 
        {
            internal_timer->start(std::bind(&SimpleTimer::simple_timer_callback, this, _1));
        }
    }

    void SimpleTimer::start(std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback)
    {
        m_update_callback = update_callback;
        m_stop_callback = stop_callback;
        
        internal_timer->start(std::bind(&SimpleTimer::simple_timer_callback, this, _1), m_stop_callback);
    }

    void SimpleTimer::start_async(std::function<void(uint64_t t_now)> update_callback)
    {
        m_update_callback = update_callback;

        if (m_stop_callback)
        {
            internal_timer->start_async(std::bind(&SimpleTimer::simple_timer_callback, this, _1), m_stop_callback);
        }
        else 
        {
            internal_timer->start_async(std::bind(&SimpleTimer::simple_timer_callback, this, _1));
        }
    }

    void SimpleTimer::start_async(std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) 
    {
        m_update_callback = update_callback;
        m_stop_callback = stop_callback;
        
        internal_timer->start_async(std::bind(&SimpleTimer::simple_timer_callback, this, _1), m_stop_callback);
    }

    void SimpleTimer::stop()
    {
        internal_timer->stop();
    }

    SimpleTimer::~SimpleTimer()
    {
        internal_timer->stop();
    }


    uint64_t SimpleTimer::get_time()
    {
        return cpm::get_time_ns();
    }

    uint64_t SimpleTimer::get_start_time()
    {
        return internal_timer->get_start_time();
    }

}
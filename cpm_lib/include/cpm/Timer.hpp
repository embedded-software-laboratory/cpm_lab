#pragma once

#include <string>
#include <functional>
#include <memory>

namespace cpm
{
    class Timer
    {
    protected:
        Timer(){}

    public:
        /**
         * \brief Create a timer that can be used for function callback
         * \param node_id ID des Timers im DDS Netzwerk
         * \param period_nanoseconds The timer is called periodically with a period of period_nanoseconds
         * \param offset_nanoseconds Initial offset (from timestamp 0)
         * \param wait_for_start For the real-time timer: Set whether the timer is started only if a start signal is sent via DDS
         * \param simulated_time_allowed Decide whether the timer can run with simulated time
         */
        static std::shared_ptr<Timer> create(
            std::string node_id,
            uint64_t period_nanoseconds, 
            uint64_t offset_nanoseconds, 
            bool wait_for_start=true,
            bool simulated_time_allowed=true
        );
        virtual void start       (std::function<void(uint64_t t_now)> update_callback) = 0;
        virtual void start_async (std::function<void(uint64_t t_now)> update_callback) = 0;
        virtual void stop() = 0;
        
        virtual uint64_t get_time() = 0;
    };
}
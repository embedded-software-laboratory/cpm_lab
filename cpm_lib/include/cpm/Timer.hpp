#pragma once

/**
 * \class Timer.hpp
 * This class calls a callback function periodically 
 * based on either the system clock or a simulated 
 * clock. The calls are synchronized in both frequency 
 * and phase to the clock.
 */

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
         * \param node_id ID of the timer in the network
         * \param period_nanoseconds The timer is called periodically with a period of period_nanoseconds
         * \param offset_nanoseconds Initial offset (from timestamp 0)
         * \param wait_for_start For the real-time timer: Set whether the timer is started only if a start signal is sent via DDS
         * \param simulated_time_allowed Decide whether the timer can run with simulated time
         */
        static std::shared_ptr<Timer> create(
            std::string node_id,
            uint64_t period_nanoseconds, 
            uint64_t offset_nanoseconds, 
            bool wait_for_start,
            bool simulated_time_allowed
        );
        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called.
         * \param update_callback the callback function
         */
        virtual void start       (std::function<void(uint64_t t_now)> update_callback) = 0;
        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * \param update_callback the callback function
         */
        virtual void start_async (std::function<void(uint64_t t_now)> update_callback) = 0;
        /**
         * \brief Stops the periodic callback and kills the thread (if it was created using start_async).
         */
        virtual void stop() = 0;
        
        /**
         * \brief Can be used to obtain the current system time in nanoseconds.
         * \return the current system time in nanoseconds
         */
        virtual uint64_t get_time() = 0;
    };
}
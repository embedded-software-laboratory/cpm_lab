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
         * \brief This function creates a Timer object and returns it via a shared pointer
         * \param node_id TODO
         * \param period_nanoseconds the callback function is called periodically each period_nanoseconds
         * \param offset_nanoseconds offset for the timer wrt the clock
         * \param simulated_time_allowed indicate whether simulated time is allowed, false for hardware interaction
         * \return returns a shared pointer which holds the timer object
         */
        static std::shared_ptr<Timer> create(
            std::string node_id,
            uint64_t period_nanoseconds, 
            uint64_t offset_nanoseconds, 
            bool simulated_time_allowed=true
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
#pragma once

/**
 * \class Timer.hpp
 * \brief This class can be used to call a callback function periodically. It also provides a timer to obtain the current system time in nanoseconds.
 * The class is created using the create function. The returned object can be used to start the thread (once) which calls the function periodically or to call the function periodically in the calling thread, and prints a warning if a timestep was missed. It can also be used to stop the thread and to obtain the current system time in nanoseconds.
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
         * \param offset_nanoseconds initial offset for the timer used for the callback
         * \param simulated_time_allowed set whether the object can be used when the time is only simulated
         * \return returns a shared pointer which holds the timer object
         */
        static std::shared_ptr<Timer> create(
            std::string node_id,
            uint64_t period_nanoseconds, 
            uint64_t offset_nanoseconds, 
            bool simulated_time_allowed=true
        );
        /**
         * \brief Start the periodic callback of the callback function in the calling thread. The thread is blocked until stop() is called in another thread to release it.
         * \param update_callback the callback function
         */
        virtual void start       (std::function<void(uint64_t t_now)> update_callback) = 0;
        /**
         * \brief Start the periodic callback of the callback function in a new thread. The calling thread is not blocked.
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
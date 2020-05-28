#pragma once

#include "cpm/Timer.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Logging.hpp"
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <thread>
#include <string>

#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/core/ddscore.hpp>

#include "cpm/exceptions.hpp"
#include "cpm/get_time_ns.hpp"

/**
 * \class SimpleTimer.hpp
 * This class calls a callback function periodically 
 * based the system clock. This timer is neither intended to work with simulated time,
 * nor is it real-time capable. Use this e.g. for timing in the GUI or other non-critical
 * timing tasks only!
 */

namespace cpm {

    class SimpleTimer : public cpm::Timer
    {
        uint64_t period_milliseconds; 
        dds::topic::Topic<ReadyStatus> ready_topic;
        dds::topic::Topic<SystemTrigger> trigger_topic;
        std::string node_id;
        dds::sub::DataReader<SystemTrigger> reader_system_trigger;

        dds::sub::cond::ReadCondition readCondition;
        dds::core::cond::WaitSet waitset;
        
        std::mutex condition_mutex; //Needs to be used for wait_for
        std::condition_variable abort_condition;
        bool active = false;
        std::thread runner_thread;
        std::function<void(uint64_t t_now)> m_update_callback;
        std::function<void()> m_stop_callback;


        bool wait_for(const std::chrono::nanoseconds wait_time); //Returns true if it was not interrupted, else false (should stop in that case)
        uint64_t receiveStartTime(); //Bool: true if start signal was received, false if stop signal was received
        bool received_stop_signal ();
        
        const bool wait_for_start; //If false, do not use receiveStartTime()
        const bool react_to_stop_signal; //If false, do not react to received stop signals

    public:
        /**
         * \brief Create a simple timer (not real-time capable) that can be used for function callback
         * \param node_id ID of the timer in the network
         * \param period_milliseconds The timer is called periodically with a period of period_milliseconds
         * \param wait_for_start Set whether the timer is started only if a start signal is sent via DDS (true), or if it should should start immediately (false)
         * \param react_to_stop_signal Set whether the timer should be stopped if a stop signal is sent within the network (optional, default is true)
         */
        SimpleTimer(std::string _node_id, uint64_t period_milliseconds, bool wait_for_start, bool react_to_stop_signal = true);
        ~SimpleTimer();

        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called.
         * \param update_callback the callback function, which in this case just gets the current time
         */
        void start       (std::function<void(uint64_t t_now)> update_callback) override;

        /**
         * Start the periodic callback of the callback function in the 
         * calling thread. The thread is blocked until stop() is 
         * called. When a stop signal is received, the stop_callback function is called (and may also call stop(), if desired).
         * This allows to user to define a custom stop behaviour, e.g. that the vehicle that uses the timer only stops driving,
         * but does not stop the internal timer.
         * \param update_callback the callback function to call when the next timestep is reached, which in this case just gets the current time
         * \param stop_callback the callback function to call when the timer is stopped
         */
        void start       (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;

        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * \param update_callback the callback function, which in this case just gets the current time
         */
        void start_async (std::function<void(uint64_t t_now)> update_callback) override;

        /**
         * Start the periodic callback of the callback function 
         * in a new thread. The calling thread is not blocked.
         * When a stop signal is received, the stop_callback function is called (and may also call stop(), if desired).
         * This allows to user to define a custom stop behaviour, e.g. that the vehicle that uses the timer only stops driving,
         * but does not stop the internal timer.
         * \param update_callback the callback function to call when the next timestep is reached, which in this case just gets the current time
         * \param stop_callback the callback function to call when the timer is stopped
         */
        void start_async (std::function<void(uint64_t t_now)> update_callback, std::function<void()> stop_callback) override;

        /**
         * \brief Stops the periodic callback and kills the thread (if it was created using start_async).
         */
        void stop() override;

        /**
         * \brief Can be used to obtain the current system time in milliseconds.
         * \return the current system time in milliseconds
         */
        uint64_t get_time() override;
    };

}
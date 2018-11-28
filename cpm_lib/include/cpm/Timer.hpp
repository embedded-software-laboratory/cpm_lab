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
        static std::shared_ptr<Timer> create(
            std::string node_id,
            uint64_t period_nanoseconds, 
            uint64_t offset_nanoseconds, 
            bool simulated_time_allowed=true
        );
        virtual void start       (std::function<void(uint64_t t_now)> update_callback) = 0;
        virtual void start_async (std::function<void(uint64_t t_now)> update_callback) = 0;
        virtual void stop() = 0;
        
        virtual uint64_t get_time() = 0;
    };
}
#include "cpm/Timer.hpp"
#include "TimerFD.hpp"

namespace cpm {

std::shared_ptr<Timer> Timer::create(
    std::string node_id,
    uint64_t period_nanoseconds, 
    uint64_t offset_nanoseconds, 
    bool simulated_time_allowed
) 
{
    // TODO switch between FD and simulated time
    return std::make_shared<TimerFD>(period_nanoseconds, offset_nanoseconds);
}


}
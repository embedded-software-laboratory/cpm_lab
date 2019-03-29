#include "cpm/Timer.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/Logging.hpp"
#include "TimerFD.hpp"
#include "TimerSimulated.hpp"

namespace cpm {

std::shared_ptr<Timer> Timer::create(
    std::string node_id,
    uint64_t period_nanoseconds, 
    uint64_t offset_nanoseconds,
    bool wait_for_start, 
    bool simulated_time_allowed
) 
{
    // Switch between FD and simulated time
    if (cpm::parameter_bool("simulated_time") && simulated_time_allowed) {
        //Use timer for simulated time
        return std::make_shared<TimerSimulated>(node_id, period_nanoseconds, offset_nanoseconds);
    }
    else if (cpm::parameter_bool("simulated_time") && !simulated_time_allowed) {
        Logging::Instance().write("Timer Error: simulated time requested but not allowed.");
        fprintf(stderr, "Error: simulated time requested but not allowed.\n");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
    else {
        //Use timer for real time
        return std::make_shared<TimerFD>(node_id, period_nanoseconds, offset_nanoseconds, wait_for_start);
    }
}


}
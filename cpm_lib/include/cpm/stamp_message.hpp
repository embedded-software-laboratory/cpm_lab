#pragma once

/**
 * \file stamp_message.hpp
 * \brief This interface provides a method that can be used to 
 * set a time stamp for a sample (for creation and validity)
 */

#include <cstdint>

namespace cpm
{
    /**
     * \brief This function takes a sample reference and 
     * sets its timestamp for the time of creation (t_now) 
     * and the timestamp for the time of validity (t_now + expected_delay) 
     * from which on the sample can be used by the receiver
     * \param message the sample whose header needs to be set
     * \param t_now the current system time in nanoseconds
     * \param expected_delay the amount of nanoseconds before the sample becomes valid (starting at t_now)
     * \ingroup cpmlib
     */
    template<typename T>
    void stamp_message(T& message, uint64_t t_now, uint64_t expected_delay)
    {
        message.header().create_stamp().nanoseconds(t_now);
        message.header().valid_after_stamp().nanoseconds(t_now + expected_delay);
    }
}
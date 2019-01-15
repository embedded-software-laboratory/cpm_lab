#pragma once

#include <cstdint>

namespace cpm
{
    template<typename T>
    void stamp_message(T& message, uint64_t t_now, uint64_t expected_delay)
    {
        message.header().create_stamp().nanoseconds(t_now);
        message.header().valid_after_stamp().nanoseconds(t_now + expected_delay);
    }
}
// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

/**
 * \class stamp_message.hpp
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
     */
    template<typename T>
    void stamp_message(T& message, uint64_t t_now, uint64_t expected_delay)
    {
        message.header().create_stamp().nanoseconds(t_now);
        message.header().valid_after_stamp().nanoseconds(t_now + expected_delay);
    }
}
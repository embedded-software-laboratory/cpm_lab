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

#include "LCCErrorLogger.hpp"

/**
 * \file LCCErrorLogger.cpp
 * \ingroup lcc
 */

LCCErrorLogger& LCCErrorLogger::Instance()
{
    static LCCErrorLogger instance;
    return instance;
}

std::string LCCErrorLogger::get_timestamp_string()
{
    auto c_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::stringstream ss;
    ss << std::put_time(std::localtime(&c_time), "%H:%M:%S");
    return ss.str();
}

void LCCErrorLogger::log_error(std::string error)
{
    std::lock_guard<std::mutex> lock(error_storage_mutex);
    std::lock_guard<std::mutex> lock2(new_error_storage_mutex);

    std::string timestamp = get_timestamp_string();

    //String is only new if it has not been added before
    if (error_storage.find(error) == error_storage.end())
    {
        new_error_storage[error] = timestamp;
    }
    error_storage[error] = timestamp;
}

std::unordered_map<std::string, std::string> LCCErrorLogger::get_all_errors()
{
    std::lock_guard<std::mutex> lock(error_storage_mutex);

    return error_storage;
}

std::unordered_map<std::string, std::string> LCCErrorLogger::get_new_errors()
{
    std::lock_guard<std::mutex> lock(new_error_storage_mutex);

    auto new_error_storage_copy = new_error_storage;
    new_error_storage.clear();

    return new_error_storage_copy;
}

void LCCErrorLogger::reset()
{
    std::lock_guard<std::mutex> lock(error_storage_mutex);
    std::lock_guard<std::mutex> lock2(new_error_storage_mutex);

    error_storage.clear();
    new_error_storage.clear();
}
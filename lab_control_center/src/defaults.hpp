#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <cstdint>
#include <stdexcept>
#include <functional>
#include <map>
#include <cstdio>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using std::vector;
using std::string;
using std::to_string;
using std::map;
using std::cout;
using std::cerr;
using std::endl;
using std::make_shared;
using std::shared_ptr;
using std::runtime_error;
using std::function;

using std::int8_t;
using std::uint8_t;
using std::int16_t;
using std::uint16_t;
using std::int32_t;
using std::uint32_t;
using std::int64_t;
using std::uint64_t;

/**
 * \file defaults.hpp
 * Contains some functions that might be used anywhere in the LCC
 * \ingroup lcc
 */

/**
 * \brief Can be used similar to printf
 * \ingroup lcc
 */
template<typename ... Args>
string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1;
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return string( buf.get(), buf.get() + size - 1 );
}

/**
 * \brief Get a random double in [0.0, 1.0] (using rand, dividing by RAND_MAX -> does not cover all possible double values)
 * \ingroup lcc
 */
double frand();
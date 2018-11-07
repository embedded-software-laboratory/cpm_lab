#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <cstdint>
#include <stdexcept>
#include <map>
#include <cstdio>

using std::vector;
using std::string;
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


uint64_t clock_gettime_nanoseconds();


// from https://stackoverflow.com/a/26221725
template<typename ... Args>
string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1;
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return string( buf.get(), buf.get() + size - 1 );
}
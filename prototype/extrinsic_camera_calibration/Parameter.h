#pragma once

#include <stdexcept>

template <typename T>
class Parameter {
private:
    T value;
    bool initialized = false;

public:
    void set(T val){
        if(initialized) throw std::runtime_error("Parameter is immutable and cannot be set again.");
        value = val;
        initialized = true;
    }
    Parameter& operator = (const Parameter&) = delete;
    void operator = (T val) {
        if(initialized) throw std::runtime_error("Parameter is immutable and cannot be set again.");
        value = val;
        initialized = true;
    }

    const T& operator()() const {
        if(!initialized) throw std::runtime_error("Parameter is not initialized.");
        return value;
    }
    T get() const {
        if(!initialized) throw std::runtime_error("Parameter is not initialized.");
        return value;
    }
    operator T() const {
        if(!initialized) throw std::runtime_error("Parameter is not initialized.");
        return value;
    }
};



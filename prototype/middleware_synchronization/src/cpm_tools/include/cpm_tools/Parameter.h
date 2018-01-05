#pragma once

#include <stdexcept>

struct ParameterUninitializedError : public std::runtime_error { ParameterUninitializedError(const std::string &name):std::runtime_error("Parameter \"" + name + "\" is not initialized."){} };
struct ParameterAlreadyInitializedError : public std::runtime_error { ParameterAlreadyInitializedError(const std::string &name):std::runtime_error("Parameter \"" + name + "\" is immutable and cannot be set again."){} };

//! \brief Stores a parameter that can only be written once.
//! Reading before writing throws an exception. Writing twice throws an exception.
//! This allows delayed construction of objects with many parameters and avoids long constructor parameter lists.
template <typename T>
class Parameter {
private:
    T value;
    bool initialized = false;
    std::string name;

public:

    explicit Parameter(const std::string &name): name(name) {}


    //! Set the parameter value.
    //! \throws ParameterAlreadyInitializedError if the parameter is already initialized.
    void set(const T &val){
        if(initialized) throw ParameterAlreadyInitializedError(name);
        value = val;
        initialized = true;
    }

    //! Set the parameter value.
    //! \throws ParameterAlreadyInitializedError if the LHS parameter is already initialized.
    //! \throws ParameterUninitializedError if the RHS parameter is not initialized.
    Parameter& operator = (const Parameter& param) {
        if(initialized) throw ParameterAlreadyInitializedError(name);
        if(!param.initialized) throw ParameterUninitializedError(name);
        value = param.value;
        initialized = true;
        return *this;
    }

    //! Set the parameter value.
    //! \throws ParameterAlreadyInitializedError if the parameter is already initialized.
    void operator = (const T &val) {
        if(initialized) throw ParameterAlreadyInitializedError(name);
        value = val;
        initialized = true;
    }

    //@{
    //! Retrieve the parameter value.
    //! \throws ParameterUninitializedError if the parameter is not initialized.
    const T& operator()() const {
        if(!initialized) throw ParameterUninitializedError(name);
        return value;
    }
    T get() const {
        if(!initialized) throw ParameterUninitializedError(name);
        return value;
    }
    operator T() const {
        if(!initialized) throw ParameterUninitializedError(name);
        return value;
    }
    //@}
};



#pragma once

#include <stdexcept>

/**
 * \brief SpecificationError
 * This class defines an error that is thrown in constructors of classes used to translate Commonroad XML files in case that the file does not meed the expected structure / specification
 */

class SpecificationError : virtual public std::runtime_error
{
public:
    SpecificationError(const std::string& msg);
};
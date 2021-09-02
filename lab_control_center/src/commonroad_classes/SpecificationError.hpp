#pragma once

#include <stdexcept>

/**
 * \brief SpecificationError
 * This class defines an error that is thrown in constructors of classes used to translate Commonroad XML files in case that the file does not meed the expected structure / specification
 * \ingroup lcc_commonroad
 */
class SpecificationError : virtual public std::runtime_error
{
public:
    /**
     * \brief Constructor for the specification error type, to create such an error
     * \param msg Error message
     */
    SpecificationError(const std::string& msg);
};
#include "commonroad_classes/SpecificationError.hpp"

/**
 * \file SpecificationError.cpp
 * \ingroup lcc_commonroad
 */

SpecificationError::SpecificationError(const std::string& msg) :
    std::runtime_error(msg)
{

}
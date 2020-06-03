#include "commonroad_classes/SpecificationError.hpp"

SpecificationError::SpecificationError(const std::string& msg) :
    std::runtime_error(msg)
{

}
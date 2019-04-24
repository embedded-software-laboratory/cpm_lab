#include "cpm/dds/Parameter.hpp"
#include <string>

/**
 * \brief To be used with ParameterStorage, to save parameter information as well as their description 
 */ 

struct ParameterWithDescription {
    Parameter parameter_data;
    std::string parameter_description;
}
#pragma once

/*
 *  ParameterDistribution.hpp
 *  This interface can be used to receive/request constant definitions of different types
 */

#include <string>
#include <vector>
#include <map>

#include "Publisher.h"
#include "Subscriber.h"

namespace cpm {
    class Parameter_distribution {
        public:
            Parameter_distribution();

            //Getter for the variables
            bool parameter_bool(std::string parameter_name);
            int32_t parameter_int(std::string parameter_name);
            double parameter_double(std::string parameter_name);
            std::string parameter_string(std::string parameter_name);
            std::vector<int32_t> parameter_ints(std::string parameter_name);
            std::vector<double> parameter_doubles(std::string parameter_name);
            std::vector<std::string> parameter_strings(std::string parameter_name);

        private:
            //Variable storage, DDS request is sent only if the storage for key 'parameter_name' is empty
            std::map<std::string, bool> param_bool;
            std::map<std::string, int32_t> param_int;
            std::map<std::string, double> param_double;
            std::map<std::string, std::string> param_string;
            std::map<std::string, std::vector<int32_t>> param_ints;
            std::map<std::string, std::vector<double>> param_doubles;
            std::map<std::string, std::vector<std::string>> param_strings;

            void requestParam(std::string parameter_name);

            Publisher<todo> publisher;
    };
}
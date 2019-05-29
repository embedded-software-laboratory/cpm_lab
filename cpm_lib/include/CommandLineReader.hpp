#pragma once

#include <string>
#include <sstream>

/**
 * \brief Used to read a single command line argument from argv
 */

namespace cpm {
    bool cmd_parameter_bool(std::string name, bool default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) != std::string::npos) {
                std::string value = param.substr(param.find("="));
                return (value == "true" || value == "True" || value == "T" || value == "1");
            }
        }

        return default_value;
    }

    int cmd_parameter_int(std::string name, int default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) != std::string::npos) {
                std::string value = param.substr(param.find("="));
                return std::stoi(value);
            }
        }

        return default_value;
    }

    std::string cmd_parameter_string(std::string name, std::string default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) != std::string::npos) {
                std::string value = param.substr(param.find("="));
                return value;
            }
        }

        return default_value;
    }
}
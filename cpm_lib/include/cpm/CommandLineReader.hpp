#pragma once

#include <string>
#include <sstream>

/**
 * \brief Used to read a single command line argument from argv
 */

namespace cpm {
    /**
     * \brief Read a boolean command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    bool cmd_parameter_bool(std::string name, bool default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) != std::string::npos) {
                std::string value = param.substr(param.find("=") + 1);
                return (value == "true" || value == "True" || value == "T" || value == "1");
            }
        }

        return default_value;
    }

    /**
     * \brief Read an integer command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    int cmd_parameter_int(std::string name, int default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) != std::string::npos) {
                std::string value = param.substr(param.find("=") + 1);

                int int_value = 0;
                try {
                    int_value = std::stoi(value);
                }
                catch (...) {
                    return default_value;
                }

                return int_value;
            }
        }

        return default_value;
    }

    /**
     * \brief Read a std::string command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    std::string cmd_parameter_string(std::string name, std::string default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) != std::string::npos) {
                std::string value = param.substr(param.find("=") + 1);
                return value;
            }
        }

        return default_value;
    }
}
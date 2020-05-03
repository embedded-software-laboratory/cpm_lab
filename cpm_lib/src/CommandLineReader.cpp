#include "cpm/CommandLineReader.hpp"

namespace cpm {

    bool cmd_parameter_bool(std::string name, bool default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) == 0) {
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
            if (param.find(key) == 0) {
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
     * \brief Read an integer command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    uint64_t cmd_parameter_uint64_t(std::string name, uint64_t default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) == 0) {
                std::string value = param.substr(param.find("=") + 1);

                unsigned long long uint_value = 0;
                try {
                    //@Janis: Alternativ lässt sich mittels stringstream casten, falls die ungleichen Typen deiner Einschätzung nach eventuell Fehler hervorrufen könnten. Dann wäre die Fehlerbehandlung jedoch auch anders.
                    uint_value = std::stoull(value);
                }
                catch (...) {
                    return default_value;
                }

                return static_cast<uint64_t>(uint_value);
            }
        }

        return default_value;
    }


    /**
     * \brief Read a double command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    double cmd_parameter_double(std::string name, double default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) == 0) {
                std::string value = param.substr(param.find("=") + 1);

                double double_value = 0;
                try {
                    double_value = std::stod(value);
                }
                catch (...) {
                    return default_value;
                }

                return double_value;
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
            if (param.find(key) == 0) {
                std::string value = param.substr(param.find("=") + 1);
                return value;
            }
        }

        return default_value;
    }

    /**
     * \brief Read an integer command line argument from argv (form: --name=value), use a default value if it does not exist
     */
    std::vector<int> cmd_parameter_ints(std::string name, std::vector<int> default_value, int argc, char *argv[]) {
        std::string key = "--" + name + "=";

        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (param.find(key) == 0) {
                std::string value = param.substr(param.find("=") + 1);

                std::vector<int> ints;
                std::stringstream id_stream(value);
                std::string single_id;
                while (std::getline(id_stream, single_id, ',')) {
                    try {
                        ints.push_back(std::stoi(single_id));
                    }
                    catch (...) {
                        return default_value;
                    }
                }

                return ints;
            }
        }

        return default_value;
    }

}
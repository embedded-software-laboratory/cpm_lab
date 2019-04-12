#pragma once

#include <string>

/**
 * \brief Used to read a single command line argument from argv
 */

struct CommandLineReader {

    bool auto_start = false;
    std::string config_file = "";

    CommandLineReader(int argc, char *argv[])
    {
        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (i + 1 < argc) {
                if (param.find("-config") != std::string::npos) {
                    ++i;
                    this->config_file = std::string(argv[i]);
                }
            }

            if (param.find("-autostart") != std::string::npos) {
                this->auto_start = true;
            }
        }
    }
};
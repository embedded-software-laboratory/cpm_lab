#pragma once

#include <string>

/**
 * \brief Used to read a single command line argument from argv
 */

struct CommandLineReader {

    bool command_auto_start = false;
    bool param_server_auto_start = false;
    std::string config_file = "";

    CommandLineReader(int argc, char *argv[], std::string default_config)
    {
        config_file = default_config;
        
        for (int i = 1; i < argc; ++i) {
            std::string param = std::string(argv[i]);
            if (i + 1 < argc) {
                if (param.find("-config") != std::string::npos) {
                    ++i;
                    this->config_file = std::string(argv[i]);
                }
            }

            if (param.find("-autostart_command") != std::string::npos) {
                this->command_auto_start = true;
            }

            if (param.find("-autostart_param") != std::string::npos) {
                this->param_server_auto_start = true;
            }
        }
    }
};
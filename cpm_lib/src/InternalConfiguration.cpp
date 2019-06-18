#include "InternalConfiguration.hpp"
#include "cpm/init.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"

namespace cpm
{
    void init(int argc, char *argv[])
    {
        InternalConfiguration::init(argc, argv);
    }


    void InternalConfiguration::init(int argc, char *argv[])
    {
        InternalConfiguration::the_instance = InternalConfiguration(
            cmd_parameter_int("dds_domain", 0, argc, argv),
            cmd_parameter_string("logging_id", "uninitialized", argc, argv)
        );

        // TODO reverse access, i.e. access the config from the logging
        cpm::Logging::Instance().set_id(InternalConfiguration::Instance().get_logging_id());
    }


    InternalConfiguration InternalConfiguration::the_instance;
}
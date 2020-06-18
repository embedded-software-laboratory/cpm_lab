#pragma once
#include <string>

namespace cpm
{
    class InternalConfiguration
    {
        static InternalConfiguration the_instance;

        
        int dds_domain = 0;
        std::string logging_id = "uninitialized";
        std::string dds_initial_peer = "";


        InternalConfiguration(){}

        InternalConfiguration(
            int _dds_domain,
            std::string _logging_id,
            std::string _dds_initial_peer
        )
        :dds_domain(_dds_domain)
        ,logging_id(_logging_id)
        ,dds_initial_peer(_dds_initial_peer)
        {}

    public:

        int get_dds_domain() { return dds_domain; }
        std::string get_logging_id() { return logging_id; }
        std::string get_dds_initial_peer() { return dds_initial_peer; }

        static void init(int argc, char *argv[]);
        static InternalConfiguration& Instance() {return the_instance;}
    };
}
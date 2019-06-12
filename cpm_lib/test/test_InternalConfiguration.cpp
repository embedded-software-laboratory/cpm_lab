#include "catch.hpp"
#include "InternalConfiguration.hpp"
#include "cpm/init.hpp"

TEST_CASE( "InternalConfiguration" ) 
{
    char program_name[] = "irrelevant_program";
    char arg1[] = "--dds_domain=31";
    char arg2[] = "--logging_id=hello";
    char *argv[] = { program_name, arg1, arg2 };
    int argc = 3;

    cpm::init(argc, argv);

    CHECK( cpm::InternalConfiguration::Instance().get_dds_domain() == 31 );
    CHECK( cpm::InternalConfiguration::Instance().get_logging_id() == "hello" );
}
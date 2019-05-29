#include "catch.hpp"
#include "cpm/CommandLineReader.hpp"

TEST_CASE( "CommandLineReader" ) {
    //Create pseudo-commandline parameters
    char program_name[] = "irrelevant_program";
    char str1[] = "--int1=7";
    char str2[] = "--bool=true";
    char str3[] = "--string=thisisastring!";
    char str4[] = "--int2=...";

    char *args[] = { program_name, str1, str2, str3, str4 };
    int argcount = 4;

    std::string expected_string = "thisisastring!";
    int expected_int_1 = 7;
    int expected_int_2 = 0;
    bool expected_bool = true;

    std::string cmd_string = cpm::cmd_parameter_string("string", "", argcount, args);
    int cmd_int_1 = cpm::cmd_parameter_int("int1", 0, argcount, args);
    int cmd_int_2 = cpm::cmd_parameter_int("int2", 0, argcount, args);
    bool cmd_bool = cpm::cmd_parameter_bool("bool", false, argcount, args);

    CHECK( expected_string == cmd_string );
    CHECK( expected_int_1 == cmd_int_1 );
    CHECK( expected_int_2 == cmd_int_2 );
    CHECK( expected_bool == cmd_bool );
}
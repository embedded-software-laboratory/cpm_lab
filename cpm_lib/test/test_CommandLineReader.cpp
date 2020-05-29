#include "catch.hpp"
#include "cpm/CommandLineReader.hpp"

TEST_CASE( "CommandLineReader" ) {
    //Create pseudo-commandline parameters
    char program_name[] = "irrelevant_program";
    char str1[] = "--int1=7";
    char str2[] = "--bool=true";
    char str3[] = "--string=thisisastring!";
    char str4[] = "--int2=...";
    char str5[] = "--list1=1,2,3,...,8";
    char str6[] = "--list2=1,2,3,4,8";
    char str7[] = "--uint1=9999999999999999999";
    char str8[] = "--double1=0.123";
    char str9[] = "--double2=-45.6";
    char str10[] = "--doubles1=..!";
    char str11[] = "--doubles2=0.314,99.998,3.3";

    char *args[] = { program_name, str1, str2, str3, str4, str5, str6, str7, str8, str9, str10, str11 };
    int argcount = 12;

    std::string expected_string = "thisisastring!";
    int expected_int_1 = 7;
    int expected_int_2 = 0;
    bool expected_bool = true;
    std::vector<int> expected_list_1{};
    std::vector<int> expected_list_2{ 1, 2, 3, 4, 8 };
    uint64_t expected_uint_1 = 9999999999999999999ull;
    double expected_double_1 = 0.123;
    double expected_double_2 = -45.6;
    std::vector<double> expected_doubles_list_1{0.0};
    std::vector<double> expected_doubles_list_2{0.314, 99.998, 3.3};

    std::string cmd_string = cpm::cmd_parameter_string("string", "", argcount, args);
    int cmd_int_1 = cpm::cmd_parameter_int("int1", 0, argcount, args);
    int cmd_int_2 = cpm::cmd_parameter_int("int2", 0, argcount, args);
    bool cmd_bool = cpm::cmd_parameter_bool("bool", false, argcount, args);
    std::vector<int> cmd_list_1 = cpm::cmd_parameter_ints("list1", expected_list_1, argcount, args);
    std::vector<int> cmd_list_2 = cpm::cmd_parameter_ints("list2", expected_list_1, argcount, args);
    uint64_t cmd_uint_1 = cpm::cmd_parameter_uint64_t("uint1", 0, argcount, args);
    double cmd_double_1 = cpm::cmd_parameter_double("double1", 0, argcount, args);
    double cmd_double_2 = cpm::cmd_parameter_double("double2", 0, argcount, args);
    std::vector<double> cmd_doubles_1 = cpm::cmd_parameter_doubles("doubles1", {0.0}, argcount, args);
    std::vector<double> cmd_doubles_2 = cpm::cmd_parameter_doubles("doubles2", {0.0}, argcount, args);

    CHECK( expected_string == cmd_string );
    CHECK( expected_int_1 == cmd_int_1 );
    CHECK( expected_int_2 == cmd_int_2 );
    CHECK( expected_bool == cmd_bool );
    CHECK( expected_list_1 == cmd_list_1 );
    CHECK( expected_list_2 == cmd_list_2 );
    CHECK( expected_uint_1 == cmd_uint_1 );
    CHECK( expected_double_1 == cmd_double_1 );
    CHECK( expected_double_2 == cmd_double_2 );
    CHECK( expected_doubles_list_1 == cmd_doubles_1);
    CHECK( expected_doubles_list_2 == cmd_doubles_2);
}

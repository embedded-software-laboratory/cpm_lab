#include "catch.hpp"
#include "cpm/Parameter.hpp"
#include "../src/ParameterServer.hpp"
#include <thread>
#include <memory>
#include <chrono>

TEST_CASE( "parameter_double" ) {

    double received_parameter_value = 0;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_double("my_param_name");
    });

    ParameterServer server;
    server.set_value("my_param_name", 42.1);

    client_thread.join();

    REQUIRE( received_parameter_value == 42.1 );
}



TEST_CASE( "parameter_strings" ) {


    const std::string str1 = "99 bottles of beer on the wall, 99 bottles of beer.";
    const std::string str2 = "Take one down and pass it around, 98 bottles of beer on the wall.";
    const std::string str3 = "98 bottles of beer on the wall, 98 bottles of beer.";
    const std::string str4 = "Take one down and pass it around, 97 bottles of beer on the wall.";

    std::vector<std::string> received_parameter_value;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_strings("my_param_name2");
    });

    ParameterServer server;
    server.set_value("my_param_name2", std::vector<std::string>{str1, str2, str3, str4});

    client_thread.join();

    REQUIRE( received_parameter_value.size() == 4 );
    REQUIRE( received_parameter_value[0] == str1 );
    REQUIRE( received_parameter_value[1] == str2 );
    REQUIRE( received_parameter_value[2] == str3 );
    REQUIRE( received_parameter_value[3] == str4 );
}
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

    std::string received_parameter_value;
    std::string received_parameter_value_2;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_string("my_param_name2");
        received_parameter_value_2 = cpm::parameter_string("my_param_name3");
    });

    ParameterServer server;
    server.set_value("my_param_name2", str1);
    server.set_value("my_param_name3", str2);

    client_thread.join();

    REQUIRE( received_parameter_value == str1 );
    REQUIRE( received_parameter_value_2 == str2 );
}

TEST_CASE( "parameter_bool" ) {

    bool received_parameter_value_true = false;
    bool received_parameter_value_false = true;

    std::thread client_thread([&](){
        received_parameter_value_true = cpm::parameter_bool("my_param_true");
        received_parameter_value_false = cpm::parameter_bool("my_param_false");
    });

    ParameterServer server;
    server.set_value("my_param_true", true);
    server.set_value("my_param_false", false);

    client_thread.join();

    REQUIRE( received_parameter_value_true );
    REQUIRE( !received_parameter_value_false );
}

#include "catch.hpp"
#include "cpm/Parameter.hpp"
#include "../src/ParameterServer.hpp"
#include <thread>
#include <memory>
#include <chrono>

TEST_CASE( "parameter" ) {

    double received_parameter_value = 0;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_double("my_param_name");
    });

    ParameterServer server;
    server.set_value("my_param_name", 42.1);

    client_thread.join();

    REQUIRE( received_parameter_value == 42.1 );
}
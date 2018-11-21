#include "catch.hpp"
#include "cpm/Parameter.hpp"
#include <thread>
#include <memory>
#include <chrono>

TEST_CASE( "parameter" ) {
    
    std::thread parameter_server_thread([](){
        // TODO create instance of parameter server,
        //      distribute parameter "my_param_name" = (int) 42
    });


    std::shared_ptr<int32_t> received_parameter_value = nullptr;

    std::thread parameter_client_thread([&](){
        int32_t value = cpm::parameter_int("my_param_name");
        received_parameter_value = std::make_shared<int32_t>(value);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // TODO kill parameter_client_thread and parameter_server_thread. 
    // This seems impossible with c++11.
    // See https://stackoverflow.com/questions/12207684/how-do-i-terminate-a-thread-in-c11
    // Maybe change the design, add a timeout?

    REQUIRE( bool(received_parameter_value) );
    REQUIRE( *received_parameter_value == 42 );
}
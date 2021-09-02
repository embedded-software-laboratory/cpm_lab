#include "catch.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/Parameter.hpp"
#include "cpm/ParameterReceiver.hpp"
#include <thread>
#include <memory>
#include <chrono>
#include <functional>
#include "cpm/get_topic.hpp"

#include "cpm/Writer.hpp"

using namespace std::placeholders;
/**
 * \brief Small helper class that creates a "dummy" parameter server for reading and writing parameters
 */
class ParameterServerDummy {
    private:
        //! DDS Writer to write parameters
        cpm::Writer<Parameter> parameter_writer;
        //! DDS Reader to read parameters async. with a callback
        cpm::AsyncReader<ParameterRequest> parameter_request_subscriber;
    public:
        /**
         * \brief Constructor, allows to register a callback for the parameter reader
         * \param callback Gets called whenever the parameter reader receives a new message
         */
        ParameterServerDummy(std::function<void(std::vector<ParameterRequest>& samples)> callback) :
            parameter_writer(
                "parameter",
                true
            ),
            parameter_request_subscriber(
                std::bind(callback, _1), 
                "parameterRequest",
                true
            )
        {

        }

        /**
         * \brief Provides access to the parameter writer
         */
        cpm::Writer<Parameter>& get_writer()
        {
            return parameter_writer;
        }
};

/**
 * \test Tests Parameters with double values
 * 
 * - If parameter requests are processed correctly by the lib
 * - If the ParameterServerDummy answers correctly
 * - If different types are supported
 * \ingroup cpmlib
 */
TEST_CASE( "parameter_double" ) {
    //Set the Logger ID
    cpm::Logging::Instance().set_id("test_parameter_double");

    //Define the parameter that should be received upon request; also make sure that no wrong data was sent
    std::string param_name = "my_param_name";
    double param_value = 42.1;
    bool received_wrong_param_name = false;

    //Thread that uses the cpm lib to request a parameter - this is supposed to be tested
    double received_parameter_value = 0;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_double(param_name);
    });

    //Requesting parameters in the cpm lib also includes its own waiting mechanism, so we skip waiting for an entity match

    //Create a callback function that acts similar to the parameter server - only send data if the expected request was received
    ParameterServerDummy server([&](std::vector<ParameterRequest>& samples){
        for (auto data : samples) {
            if (data.name() == param_name) {
                Parameter param = Parameter();
                param.name(param_name);

                std::vector<double> stdDoubles;
                stdDoubles.push_back(param_value);
                rti::core::vector<double> doubles(stdDoubles);

                param.type(ParameterType::Double);
                param.values_double(doubles);
                server.get_writer().write(param);
            }
            else {
                received_wrong_param_name = true;
            }
        }
    });

    client_thread.join();

    REQUIRE( received_parameter_value == param_value );
    CHECK( ! received_wrong_param_name );
}


/**
 * \test Tests Parameters with string values
 * 
 * - If parameter requests are processed correctly by the lib
 * - If the ParameterServerDummy answers correctly
 * - If different types are supported
 * \ingroup cpmlib
 */
TEST_CASE( "parameter_strings" ) {
    //Set the Logger ID
    cpm::Logging::Instance().set_id("test_parameter_strings");

    //Test two different string types to test both overloads of cpm::parameter_string
    std::string param_name_1 = "param_name_1";
    const std::string string_param_1 = "99 bottles of beer on the wall, 99 bottles of beer.";
    std::string param_name_2 = "param_name_2";
    const char* string_param_2 = "Take one down and pass it around, 98 bottles of beer on the wall.";

    //Thread to request parameters via the cpm lib
    std::string received_parameter_value;
    std::string received_parameter_value_2;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_string(param_name_1);
        received_parameter_value_2 = cpm::parameter_string(param_name_2);
    });

    //Requesting parameters in the cpm lib also includes its own waiting mechanism, so we skip waiting for an entity match

    //Create a callback function that acts similar to the parameter server - only send data if the expected request was received
    ParameterServerDummy server([&](std::vector<ParameterRequest>& samples){
        for (auto data : samples) {
            if (data.name() == param_name_1) {
                Parameter param = Parameter();
                param.name(param_name_1);
                param.type(ParameterType::String);
                param.value_string(string_param_1);
                server.get_writer().write(param);
            }
            else if (data.name() == param_name_2) {
                Parameter param = Parameter();
                param.name(param_name_2);
                param.type(ParameterType::String);
                param.value_string(std::string(string_param_2));
                server.get_writer().write(param);
            }
        }
    });

    client_thread.join();

    REQUIRE( received_parameter_value == string_param_1 );
    REQUIRE( received_parameter_value_2 == string_param_2 );
}

/**
 * \test Tests Parameters with boolean values
 * 
 * - If parameter requests are processed correctly by the lib
 * - If the ParameterServerDummy answers correctly
 * - If different types are supported
 * \ingroup cpmlib
 */
TEST_CASE( "parameter_bool" ) {
    //Set the Logger ID
    cpm::Logging::Instance().set_id("test_parameter_bool");

    //Bool parameters that are supposed to be tested; initialized with the opposite of their desired value
    std::string param_name_1 = "my_param_true";
    bool received_parameter_value_true = false;
    bool desired_paramater_value_true = true;
    std::string param_name_2 = "my_param_false";
    bool received_parameter_value_false = true;
    bool desired_paramater_value_false = false;

    //Thread to request parameters via the cpm lib
    std::thread client_thread([&](){
        received_parameter_value_true = cpm::parameter_bool(param_name_1);
        received_parameter_value_false = cpm::parameter_bool(param_name_2);
    });

    //Requesting parameters in the cpm lib also includes its own waiting mechanism, so we skip waiting for an entity match

    //Create a callback function that acts similar to the parameter server - only send data if the expected request was received
    ParameterServerDummy server([&](std::vector<ParameterRequest>& samples){
        for (auto data : samples) {
            if (data.name() == param_name_1) {
                Parameter param = Parameter();
                param.name(param_name_1);
                param.type(ParameterType::Bool);
                param.value_bool(desired_paramater_value_true);
                server.get_writer().write(param);
            }
            else if (data.name() == param_name_2) {
                Parameter param = Parameter();
                param.name(param_name_2);
                param.type(ParameterType::Bool);
                param.value_bool(desired_paramater_value_false);
                server.get_writer().write(param);
            }
        }
    });

    client_thread.join();

    REQUIRE( received_parameter_value_true );
    REQUIRE( !received_parameter_value_false );
}

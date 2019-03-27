#include "catch.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/AsyncReader.hpp"
#include "cpm/Parameter.hpp"
#include "../src/ParameterReceiver.hpp"
#include <thread>
#include <memory>
#include <chrono>
#include <functional>

/**
 * Tests:
 * - If parameter requests are processed correctly by the lib
 * - If the ParameterServer answers correctly
 * - If different types are supported
 */

using namespace std::placeholders;
class ParameterServer {
    private:
        dds::pub::DataWriter<Parameter> parameter_writer;
        cpm::AsyncReader<ParameterRequest> parameter_request_subscriber;
    public:
        ParameterServer(std::function<void(dds::pub::DataWriter<Parameter>&, dds::sub::LoanedSamples<ParameterRequest>& samples)> callback) :
            parameter_writer(
                dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), 
                ParameterReceiver::Instance().parameterTopic
            ),
            parameter_request_subscriber(
                std::bind(callback, parameter_writer, _1), 
                cpm::ParticipantSingleton::Instance(), 
                ParameterReceiver::Instance().parameterRequestTopic
            )
        {

        }
};

TEST_CASE( "parameter_double" ) {

    //Define the parameter that should be received upon request; also make sure that no wrong data was sent
    std::string param_name = "my_param_name";
    double param_value = 42.1;
    bool received_wrong_param_name = false;

    //Thread that uses the cpm lib to request a parameter - this is supposed to be tested
    double received_parameter_value = 0;
    std::thread client_thread([&](){
        received_parameter_value = cpm::parameter_double(param_name);
    });

    //Create a callback function that acts similar to the parameter server - only send data if the expected request was received
    ParameterServer server([&](dds::pub::DataWriter<Parameter>& writer, dds::sub::LoanedSamples<ParameterRequest>& samples){
        for (auto sample : samples) {
            if (sample.info().valid()) {
                if (sample.data().name() == param_name) {
                    Parameter param = Parameter();
                    param.name(param_name);

                    std::vector<double> stdDoubles;
                    stdDoubles.push_back(param_value);
                    rti::core::vector<double> doubles(stdDoubles);

                    param.type(ParameterType::Double);
                    param.values_double(doubles);
                    writer.write(param);
                }
                else {
                    received_wrong_param_name = true;
                }
            }
        }
    });

    client_thread.join();

    REQUIRE( received_parameter_value == param_value );
    CHECK( ! received_wrong_param_name );
}



TEST_CASE( "parameter_strings" ) {
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

    //Create a callback function that acts similar to the parameter server - only send data if the expected request was received
    ParameterServer server([&](dds::pub::DataWriter<Parameter>& writer, dds::sub::LoanedSamples<ParameterRequest>& samples){
        for (auto sample : samples) {
            if (sample.info().valid()) {
                if (sample.data().name() == param_name_1) {
                    Parameter param = Parameter();
                    param.name(param_name_1);
                    param.type(ParameterType::String);
                    param.value_string(string_param_1);
                    writer.write(param);
                }
                else if (sample.data().name() == param_name_2) {
                    Parameter param = Parameter();
                    param.name(param_name_2);
                    param.type(ParameterType::String);
                    param.value_string(std::string(string_param_2));
                    writer.write(param);
                }
            }
        }
    });

    client_thread.join();

    REQUIRE( received_parameter_value == string_param_1 );
    REQUIRE( received_parameter_value_2 == string_param_2 );
}

TEST_CASE( "parameter_bool" ) {
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

    //Create a callback function that acts similar to the parameter server - only send data if the expected request was received
    ParameterServer server([&](dds::pub::DataWriter<Parameter>& writer, dds::sub::LoanedSamples<ParameterRequest>& samples){
        for (auto sample : samples) {
            if (sample.info().valid()) {
                if (sample.data().name() == param_name_1) {
                    Parameter param = Parameter();
                    param.name(param_name_1);
                    param.type(ParameterType::Bool);
                    param.value_bool(desired_paramater_value_true);
                    writer.write(param);
                }
                else if (sample.data().name() == param_name_2) {
                    Parameter param = Parameter();
                    param.name(param_name_2);
                    param.type(ParameterType::Bool);
                    param.value_bool(desired_paramater_value_false);
                    writer.write(param);
                }
            }
        }
    });

    client_thread.join();

    REQUIRE( received_parameter_value_true );
    REQUIRE( !received_parameter_value_false );
}

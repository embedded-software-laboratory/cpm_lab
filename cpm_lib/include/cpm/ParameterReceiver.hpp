#pragma once

/*
 * Singleton that receives and stores constants, e.g. for configuration
 * Is used by to get data for the user (requests data from the param server or uses stored data if available)
 */

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "dds/Parameter.hpp"
#include "dds/ParameterRequest.hpp"
#include "cpm/Logging.hpp"

#include "cpm/AsyncReader.hpp"
#include <dds/pub/ddspub.hpp>

namespace cpm
{

    class ParameterReceiver {
    public:
        static ParameterReceiver& Instance();

        //Delete move and copy op
        ParameterReceiver(ParameterReceiver const&) = delete;
        ParameterReceiver(ParameterReceiver&&) = delete; 
        ParameterReceiver& operator=(ParameterReceiver const&) = delete;
        ParameterReceiver& operator=(ParameterReceiver &&) = delete;

        //Provide access similar to the interface
        bool parameter_bool(std::string parameter_name);
        int32_t parameter_int(std::string parameter_name);
        double parameter_double(std::string parameter_name);
        std::string parameter_string(std::string parameter_name);
        std::vector<int32_t> parameter_ints(std::string parameter_name);
        std::vector<double> parameter_doubles(std::string parameter_name);

    private:
        /**
         * \brief Constructor. Creates a reliable DataWriter and uses the "is_reliable" parameter of the AsyncReader to create a reliable DataReader as well. Also binds the callback function / passes it to the AsyncReader.
         */
        ParameterReceiver();

        //Variable storage, DDS request is sent only if the storage for key 'parameter_name' is empty
        std::map<std::string, bool> param_bool;
        std::map<std::string, int32_t> param_int;
        std::map<std::string, double> param_double;
        std::map<std::string, std::string> param_string;
        std::map<std::string, std::vector<int32_t>> param_ints;
        std::map<std::string, std::vector<double>> param_doubles;

        //Mutex for each map
        std::mutex param_bool_mutex;
        std::mutex param_int_mutex;
        std::mutex param_double_mutex;
        std::mutex param_string_mutex;
        std::mutex param_ints_mutex;
        std::mutex param_doubles_mutex;

        /**
         * \brief Sends a param request to the param server with the given parameter name
         * \param parameter_name Name of the requested param
         */
        void requestParam(std::string parameter_name);

        /**
         * \brief Callback function that handles incoming parameter definitions. Parameters are stored in maps depending on their type and name for later access.
         * \param samples Samples to be processed by the callback function (received messages)
         */
        void callback(dds::sub::LoanedSamples<Parameter>& samples);

    public:
        dds::topic::Topic<Parameter> parameterTopic;
        dds::topic::Topic<ParameterRequest> parameterRequestTopic;

    private:
        dds::pub::DataWriter<ParameterRequest> writer;
        cpm::AsyncReader<Parameter> subscriber;
    };

}
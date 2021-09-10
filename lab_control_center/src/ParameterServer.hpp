#pragma once

#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <atomic>

#include "Parameter.hpp"
#include "ParameterRequest.hpp"
#include "ParameterStorage.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/Writer.hpp"
#include "ParameterWithDescription.hpp"

#include "cpm/AsyncReader.hpp"

/**
 * \class ParameterServer
 * \brief Server that distributes parameter values. This server uses ParameterStorage to store its values.
 * \ingroup lcc
*/
class ParameterServer {
private:    
    /**
     * \brief Callback for the async reader, readerParameterRequest
     * \param samples Messages read by the async reader
     */
    void handleParamRequest(std::vector<ParameterRequest>& samples);

    /**
     * \brief Called by handleParamRequest for each message received by the async reader. 
     * Looks up in storage if a parameter with name exists (first checks for bool params, then for uint64_t etc.).
     * If so, it is sent to the network using writer.
     * Else, the message is ignored.
     * \param name The parameter name
     */
    void handleSingleParamRequest(std::string name);

    //Communication
    //! DDS Writer to send parameter values to the network, if set by the user or requested
    cpm::Writer<Parameter> writer;
    //! DDS Async Reader that looks for parameter requests and calls handleParamRequest if new messages were received
    cpm::AsyncReader<ParameterRequest> readerParameterRequest;

    //! Thread to send parameters in the network not immediately, but 5 seconds after the ParameterServer object was created
    std::thread delayed_init_param_thread;

public:
    /**
     * \brief Constructor
     * \param _storage The parameter storage to be used by the server, to send parameter values. These are updated in the UI, so the access to the storage is shared.
     */
    ParameterServer(std::shared_ptr<ParameterStorage> _storage);

    /**
     * \brief Deconstructor, to clear the delayed_init_param_thread
     */
    ~ParameterServer();

    /**
     * \brief Function to send all parameters to the network, also used by the constructor
     */
    void resend_all_params();

    /**
     * \brief Called by parameter storage whenever a parameter value was changed by the user, to allow updates of the parameter value within the network
     */
    void resend_param_callback(std::string name);
    
    //! Shared parameter storage
    std::shared_ptr<ParameterStorage> storage;
};
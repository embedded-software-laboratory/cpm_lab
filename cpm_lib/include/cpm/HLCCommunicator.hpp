// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

// Set to true to get additional information about execution time in stdout
#define TIMED true

// General C++ imports
#include <functional>                       // So we can use std::function
#include <limits>                           // To get maximum integer value (for stop condition)
#include <future>                           // So we can use std::async, std::future etc
#include <chrono>                           // For sleep duration
#include <thread>                           // For sleep
#include <sstream>                          // For std::stringstream

// cpm_lib
#include "cpm/get_topic.hpp"
#include "cpm/Writer.hpp"
#include "cpm/ReaderAbstract.hpp"
#include "cpm/Participant.hpp"
#include "cpm/Logging.hpp"

// DDS topics
#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"
#include "VehicleStateList.hpp"
#include "StopRequest.hpp"

class HLCCommunicator{
    /**
     * \class HLCCommunicator
     * \brief This class handles communication of a High Level Controller
     * with the Middleware. Thus it also dictates the timing of when the HLC
     * is supposed to plan a timestep.
     * \ingroup cpmlib
     */

    //! Vehicle ID; needed for the ready message 
    int vehicle_id;

    //! Participant to communicate with the middleware
    std::shared_ptr<cpm::Participant> p_local_comms_participant;

    //! SystemTrigger value that means "stop" (as defined in SystemTrigger.idl)
    const uint64_t trigger_stop = std::numeric_limits<uint64_t>::max();

    //! Is only true on the first timestep
    bool first_timestep = true;

    //! Wether we've received a new VehicleStateList, so we only read it once
    bool new_vehicleStateList = false;

    //! The latest VehicleStateList we have received
    VehicleStateList vehicle_state_list;

    //! Writer to write a ReadyStatus message to Middleware (to signal we can start)
    cpm::Writer<ReadyStatus>    writer_readyStatus;
    //! Writer to write a StopRequest to Middleware (currently unused)
    cpm::Writer<StopRequest>    writer_stopRequest;
    //! Reader to read VehicleStateList messages from Middleware (for timing)
    cpm::ReaderAbstract<VehicleStateList>   reader_vehicleStateList;
    //! Reader to read SystemTrigger messages from Middleware (for stop signal)
    cpm::ReaderAbstract<SystemTrigger>      reader_systemTrigger;

    //! Callback function for when we need to take on the first timestep
    std::function<void(VehicleStateList)>   on_first_timestep;
    //! Callback function for when we need to take every timestep (including the first one)
    std::function<void(VehicleStateList)>   on_each_timestep;
    //! Callback function for when we need to cancel a planning timestep before it's finished
    std::function<void()>                   on_cancel_timestep;
    //! Callback function for when we have to completely stop planning
    std::function<void()>                   on_stop;

    //! Future object to check if onEachTimestep has finished yet
    std::future<void> planning_future;

    /**
     * \brief Run a timestep
     */
    void runTimestep();

    /**
     * \brief Send a ready status message to middleware
     * Sends a ReadyStatus message to the middleware, containing an arbitray TimeStamp,
     * and an identifier for our HLC
     */
    void sendReadyMessage();

    /**
     * \brief Check if we need to stop
     * Check, if we have received a SystemTrigger with the maximum value of a uint64
     * in the next_start field. That means we need to stop.
     */
    bool stopSignalReceived();

    /**
     * \brief Writes a short summary of the setup to Logging
     * Includes vehicle_id, and which callbacks are defined or not defined
     */
    void writeInfoMessage();

public:

    /**
     * \brief Constructor for HLCCommunicator
     */
    HLCCommunicator(int _vehicle_id, int middleware_domain, std::string qos_file, std::string qos_profile);

    /**
     * \brief Returns a participant that can communicate with the middleware
     *
     * This can be used to e.g. create a writer to write VehicleCommandTrajectories
     */
    std::shared_ptr<cpm::Participant> getLocalParticipant(){ return p_local_comms_participant; };

    /**
     * \brief Additional steps that need to be taken on the first timestep
     * \param TODO
     *
     * Used for initial setup, that couldn't be done earlier.
     */
    void onFirstTimestep(std::function<void(VehicleStateList)> callback) { on_first_timestep = callback; };

    /**
     * \brief TODO
     * \param TODO
     */
    void onEachTimestep(std::function<void(VehicleStateList)> callback) { on_each_timestep = callback; };

    /**
     * \brief TODO
     * \param TODO
     */
    void onCancelTimestep(std::function<void()> callback) { on_cancel_timestep = callback; };

    /**
     * \brief TODO
     * \param TODO
     */
    void onStop(std::function<void()> callback) { on_stop = callback; };

    /**
     * \brief Communicate that we're ready and start planning
     */
    void start();
};

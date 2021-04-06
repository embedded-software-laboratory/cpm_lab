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

#pragma once
#include <string>

namespace cpm
{
    /**
     * \class InternalConfiguration
     * \brief This class sets up the DDS domain, logging ID and DDS initial peer, and its init function 
     * should be used whenever the cpm library is used, at the start of the program
     * \ingroup cpmlib
     */
    class InternalConfiguration
    {
    private: 
    
        //! The actual Singleton instance
        static InternalConfiguration the_instance;

        //! DDS domain for the participant singleton, so for most of the communication
        int dds_domain = 0;
        //! ID for log messages, usually program type, e.g. "LCC" or "middleware"
        std::string logging_id = "uninitialized";
        //! Initial DDS peer, usually the LCC main computer (network performance reasons)
        std::string dds_initial_peer = "";

        /**
         * \brief Empty default constructor, private, can / should not be used
         */
        InternalConfiguration(){}

        /**
         * \brief Constructor, private. Used internally by the Instance() function to create the singleton.
         * \param _dds_domain DDS Domain for the Participant Singleton
         * \param _logging_id Logging ID for the Logger
         * \param _dds_initial_peer Set initial peer(s) for the DDS communication
         */
        InternalConfiguration(
            int _dds_domain,
            std::string _logging_id,
            std::string _dds_initial_peer
        )
        :dds_domain(_dds_domain)
        ,logging_id(_logging_id)
        ,dds_initial_peer(_dds_initial_peer)
        {}

    public:

        /**
         * \brief Get the set DDS Domain
         */
        int get_dds_domain() { return dds_domain; }

        /**
         * \brief Get the set logging ID
         */
        std::string get_logging_id() { return logging_id; }

        /**
         * \brief Get the set initial DDS peers
         */
        std::string get_dds_initial_peer() { return dds_initial_peer; }

        /**
         * \brief Init function that should be called at the start of every program that uses the cpm lib
         * Initializes the Singleton and values used by other parts of the library, which are read from the command line:
         * --dds_domain
         * --dds_initial_peer
         * --logging_id
         */
        static void init(int argc, char *argv[]);

        /**
         * \brief Get access to the internal configuration Singleton, should mostly / only be used internally for constructing other parts of the library
         */
        static InternalConfiguration& Instance() {return the_instance;}
    };
}
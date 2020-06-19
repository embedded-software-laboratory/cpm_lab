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
    class InternalConfiguration
    {
        static InternalConfiguration the_instance;

        
        int dds_domain = 0;
        std::string logging_id = "uninitialized";
        std::string dds_initial_peer = "";


        InternalConfiguration(){}

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

        int get_dds_domain() { return dds_domain; }
        std::string get_logging_id() { return logging_id; }
        std::string get_dds_initial_peer() { return dds_initial_peer; }

        static void init(int argc, char *argv[]);
        static InternalConfiguration& Instance() {return the_instance;}
    };
}
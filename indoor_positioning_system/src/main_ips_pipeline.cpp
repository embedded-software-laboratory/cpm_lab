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

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include "cpm/AsyncReader.hpp"
#include "LedPoints.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/Logging.hpp"
#include "cpm/init.hpp"
#include "IpsPipeline.hpp"



int main(int argc, char* argv[])
{
    if(argc < 2) {
        std::cout << "To enable visualization use parameter --visualization=1" << std::endl;
    }

    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("ips_pipeline");

    const bool enable_visualization = cpm::cmd_parameter_bool("visualization", false, argc, argv);
    IpsPipeline ipsPipeline(enable_visualization);


    cpm::AsyncReader<LedPoints> ipsLedPoints_reader(
        [&](dds::sub::LoanedSamples<LedPoints>& samples){
            for(auto sample : samples) 
                if(sample.info().valid()) 
                    ipsPipeline.apply(sample.data());
        }, 
        cpm::ParticipantSingleton::Instance(), 
        cpm::get_topic<LedPoints>("ipsLedPoints")
    );

    if(argc > 1 && std::string(argv[1]) == "replay")
    {
        // stop previous replay
        system("killall -9 rtireplay");

        // start replay
        system("rtireplay -cfgName mydefault -cfgFile recordings/replay_config.xml");
    }
    else
    {
        while(1) sleep(1);
    }

    exit(0);
    return 0;
}
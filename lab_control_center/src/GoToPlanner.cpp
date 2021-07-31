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

#include "GoToPlanner.hpp"

#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "MatlabEngine.hpp"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include "MatlabDataArray.hpp"
#pragma GCC diagnostic pop

using namespace matlab::engine;


GoToPlanner::GoToPlanner(){}

GoToPlanner::GoToPlanner(std::function<std::vector<double>()> get_goal_poses)
: get_goal_poses(get_goal_poses)
{
}

void GoToPlanner::go_to_start_poses(std::vector<uint8_t> vehicle_ids)
{
    std::cout << "Going to formation ..." << std::endl;
    

    // dummy inputs to test
    std::vector<double> goal_poses = get_goal_poses();

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Create MATLAB data array factory
    matlab::data::ArrayFactory factory;

    // Create args vector containing 2 arrays
    std::vector<matlab::data::Array> args({
        factory.createArray<uint8_t>({1, vehicle_ids.size()}, vehicle_ids.data(), vehicle_ids.data()+vehicle_ids.size()),
        factory.createArray<double>({3, vehicle_ids.size()}, goal_poses.data(), goal_poses.data()+goal_poses.size())
    });
    // Call MATLAB function
    // TODO use absolute_executable_path
    matlabPtr->eval(u"addpath('../tools/go_to_formation/matlab/');");
    FutureResult<std::vector<matlab::data::Array>> future = matlabPtr->fevalAsync(u"go_to_formation", 0, args);
    // matlabPtr->feval(u"go_to_formation", 0, args);
    // std::vector<matlab::data::Array> results = future.get();

    return;
}
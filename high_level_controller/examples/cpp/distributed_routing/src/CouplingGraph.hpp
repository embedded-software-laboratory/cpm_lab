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

#include <vector>
#include <map>
#include <set>
#include <cstdint>
#include <cassert>

// For printing internals
#include <string>
#include <sstream>

using std::vector;
using std::set;
using std::pair;
using std::map;
using std::string;

class CouplingGraph
{
    set<uint8_t> vehicleSet;

    /* Defines how a vehicle is supposed to treat other vehicles.
     * previous means, the other vehicle needs to finish planning before we do.
     * concurrent_iterative means, that we go back and forth with planning this.
     * ignore means, that we don't care about this cars planned trajectory.
     *  We still have to wait for its result though.
     */
    enum CouplingType { previousVehicle, concurrentVehicle, ignoredVehicle };
    map<uint8_t, map<uint8_t, CouplingType>> couplingData;
    
    void setPreviousVehiclesToDefault(uint8_t vehicleId);
    set<uint8_t> getMatchingVehicles(uint8_t vehicleId, CouplingType type);
    set<uint8_t> vectorToSet(vector<uint8_t> vector);

    public:
        CouplingGraph(){};
        CouplingGraph(vector<uint8_t> vehicleIds);
        // Alternative constructor, because not much else uses uint8_t
        CouplingGraph(vector<int> vehicleIds);
        
        void setDefaultOrder();
        void addIterativeBlock(vector<int> vehicleIds);

        set<uint8_t> getPreviousVehicles(uint8_t vehicleId);
        set<uint8_t> getConcurrentVehicles(uint8_t vehicleId);
        set<uint8_t> getIgnoredVehicles(uint8_t vehicleId);
        set<uint8_t> getVehicles();

        void setParallelPlanningMode(uint8_t mode);
        uint8_t getParallelPlanningMode();

        string toString();
};

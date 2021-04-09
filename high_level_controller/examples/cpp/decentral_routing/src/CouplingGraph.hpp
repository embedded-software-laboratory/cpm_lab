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

class CouplingGraph
{
    std::set<uint8_t> vehicleSet;
    std::map<uint8_t,std::set<uint8_t>> couplingData;
    
    void setPreviousVehiclesToDefault(uint8_t vehicleId);
    std::set<uint8_t> vectorToSet(std::vector<uint8_t> vector);

    public:
        CouplingGraph(){};
        CouplingGraph(
               std::vector<uint8_t> vehicleIds,
               bool useDefaultOrder = 1);
        // Alternative constructor, because nothing else uses uint8_t
        CouplingGraph(
               std::vector<int> vehicleIds,
               bool useDefaultOrder = 1);

        void setPreviousVehicles(std::map<uint8_t, std::vector<uint8_t>> data);
        void setPreviousVehicles(uint8_t vehicleId, std::vector<uint8_t> previousVehicles);
        std::set<uint8_t> getPreviousVehicles(uint8_t vehicleId);
        std::set<uint8_t> getVehicles();
};
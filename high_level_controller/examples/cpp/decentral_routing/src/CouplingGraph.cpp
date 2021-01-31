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

#include "CouplingGraph.hpp"

CouplingGraph::CouplingGraph(std::vector<uint8_t> vehicleIds, bool useDefaultOrder){
    vehicleSet = vectorToSet(vehicleIds);

    if( useDefaultOrder ) {
        for( uint8_t vehicleId : vehicleSet ) { 
            setPreviousVehiclesToDefault(vehicleId);
        }
    }
}

CouplingGraph::CouplingGraph(std::vector<int> vehicleIds, bool useDefaultOrder) :
    CouplingGraph(std::vector<uint8_t>(vehicleIds.begin(), vehicleIds.end()), useDefaultOrder){}

void CouplingGraph::setPreviousVehicles(std::map<uint8_t, std::vector<uint8_t>> data) {
    for( auto element : data ) {
        couplingData[element.first] = vectorToSet(element.second);
    }
} 

void CouplingGraph::setPreviousVehicles(uint8_t vehicleId, std::vector<uint8_t> prevVehiclesVector) {
    couplingData[vehicleId] = vectorToSet(prevVehiclesVector);
}

std::set<uint8_t> CouplingGraph::getPreviousVehicles(uint8_t vehicleId) {
    // vehicleId needs to be part of vehicleList
    assert(vehicleSet.find(vehicleId) != vehicleSet.end());

    return couplingData[vehicleId];
}

std::set<uint8_t> CouplingGraph::getVehicles() {
    return vehicleSet;
}

void CouplingGraph::setPreviousVehiclesToDefault(uint8_t vehicleId) {
    // vehicleId needs to be part of vehicleSet
    assert(vehicleSet.find(vehicleId) != vehicleSet.end());

    std::set<uint8_t> previousVehicles;
    // Default: Wait for all vehicles before you in the list
    // The same would be achieved by just waiting for the directly preceding vehicle
    for( uint8_t otherVehicleId : vehicleSet ) {
        if( otherVehicleId < vehicleId ) {
            previousVehicles.insert(otherVehicleId);
        } else {
            continue;
        }
    }
    couplingData[vehicleId] = previousVehicles;
}

std::set<uint8_t> CouplingGraph::vectorToSet(std::vector<uint8_t> vector) {
    std::set<uint8_t> set;
    for( auto e : vector ) {
        set.insert(e);
    }
    return set;
}

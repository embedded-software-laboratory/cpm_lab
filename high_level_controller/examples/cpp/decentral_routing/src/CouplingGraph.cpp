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

/*
 * DefaultOrder bedeutet, dass Fahrzeuge mit niedrigeren IDs
 * früher Planen und auch höhere Priorität haben.
 */
CouplingGraph::CouplingGraph(vector<uint8_t> vehicleIds){
    vehicleSet = vectorToSet(vehicleIds);
    setDefaultOrder();
}

CouplingGraph::CouplingGraph(vector<int> vehicleIds) :
    CouplingGraph(vector<uint8_t>(vehicleIds.begin(), vehicleIds.end())){}

void CouplingGraph::setDefaultOrder(){
    assert(vehicleSet.size() != 0);
    
    couplingData.clear();
    for( auto const &vehicleId : vehicleSet ){
        map<uint8_t, CouplingType> vehicleData;
        for( auto const &otherVehicle : vehicleSet ){
            if( otherVehicle < vehicleId ) {
                vehicleData.insert(std::make_pair(otherVehicle, previousVehicle));
            } else {
                vehicleData.insert(std::make_pair(otherVehicle, ignoredVehicle));
            }
        }
        couplingData.insert({vehicleId, vehicleData});
    }
}

void CouplingGraph::addIterativeBlock(vector<int> blockIds){
    //TODO: Check if blockIds are actually a block, i.e. vehicles that are next to each other on the graph

    for( auto const &vehicleId1 : blockIds ){
        // Vehicles need to be inside our vehicleSet 
        assert(vehicleSet.find(vehicleId1) != vehicleSet.end());
        auto &vehicleData = couplingData.at(vehicleId1);
        for( auto const &vehicleId2 : blockIds ){
            vehicleData[vehicleId2] = concurrentVehicle;
        }
    }
}

set<uint8_t> CouplingGraph::getPreviousVehicles(uint8_t vehicleId) {
    return getMatchingVehicles(vehicleId, previousVehicle);
}

set<uint8_t> CouplingGraph::getConcurrentVehicles(uint8_t vehicleId) {
    return getMatchingVehicles(vehicleId, concurrentVehicle);
}

set<uint8_t> CouplingGraph::getIgnoredVehicles(uint8_t vehicleId) {
    return getMatchingVehicles(vehicleId, ignoredVehicle);
}

set<uint8_t> CouplingGraph::getVehicles() {
    return vehicleSet;
}

set<uint8_t> CouplingGraph::getMatchingVehicles(uint8_t vehicleId, CouplingType type) {
    // vehicleId needs to be part of vehicleList
    assert(vehicleSet.find(vehicleId) != vehicleSet.end());

    auto vehicleData = couplingData.at(vehicleId);

    set<uint8_t> result;

    for( auto const &entry : vehicleData ) {
        if ( entry.second == type ) {
           result.insert(entry.first);
        } 
    }

    return result;
}

set<uint8_t> CouplingGraph::vectorToSet(vector<uint8_t> vector) {
    set<uint8_t> set;
    for( auto e : vector ) {
        set.insert(e);
    }
    return set;
}

string CouplingGraph::toString() {
    std::stringstream ss;  

    for(auto const &entry1 : couplingData){
        auto vehicleId   = entry1.first;
        auto vehicleData = entry1.second;
        ss << "Data for vehicle " << static_cast<uint32_t>(vehicleId) << " : ";
        for(auto const &entry2 : vehicleData) {
            ss << entry2.second << ", ";
        }
        ss << std::endl;
    }
    return ss.str();
}

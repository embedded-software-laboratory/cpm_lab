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
#include "lane_graph.hpp"
#include "Pose2D.hpp"
#include <vector>
using std::vector;


class LaneGraphTools : public LaneGraph
{
    std::vector<std::vector<double>> edges_s;
public:
    std::vector< std::vector< std::vector< std::vector<bool> > > > edge_path_collisions;
    LaneGraphTools();
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index) const;
    vector<size_t> find_subsequent_edges(int edge_index) const;
    void move_along_route(vector<size_t> route_edge_indices, size_t &edge_index, size_t &edge_path_index, double &delta_s) const;
};


extern const LaneGraphTools laneGraphTools;
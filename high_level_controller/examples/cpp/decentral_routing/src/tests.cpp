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

#include "lane_graph_tools.hpp"
#include <iostream>

/**
 * \file tests.cpp
 * \ingroup decentral_routing
 */

int main()
{
    for (size_t i_start_edge = 0; i_start_edge < laneGraphTools.n_edges; ++i_start_edge)
    {
        vector<size_t> current_route_edge_indices;
        current_route_edge_indices.push_back(i_start_edge);
        while(current_route_edge_indices.size() < 500)
        {
            auto next_edges = laneGraphTools.find_subsequent_edges(current_route_edge_indices.back());
            assert(next_edges.size() > 0);
            current_route_edge_indices.push_back(next_edges.at(0));
        }


        size_t current_edge_index_A = current_route_edge_indices[0];
        size_t current_edge_path_index_A = 3;
        double delta_s_A = 2.0;

        laneGraphTools.move_along_route
        (
            current_route_edge_indices, 
            current_edge_index_A, 
            current_edge_path_index_A, 
            delta_s_A
        );


        size_t current_edge_index_B = current_route_edge_indices[0];
        size_t current_edge_path_index_B = 3;
        double delta_s_B = 1.0;

        laneGraphTools.move_along_route
        (
            current_route_edge_indices, 
            current_edge_index_B, 
            current_edge_path_index_B, 
            delta_s_B
        );

        delta_s_B += 1.0;

        // delete old route edge(s)
        while( !current_route_edge_indices.empty()
            && current_route_edge_indices.at(0) != current_edge_index_B)
        {
            current_route_edge_indices.erase(current_route_edge_indices.begin());
        }

        laneGraphTools.move_along_route
        (
            current_route_edge_indices, 
            current_edge_index_B, 
            current_edge_path_index_B, 
            delta_s_B
        );

        if(current_edge_path_index_A != current_edge_path_index_B)
        {
            std::cout << current_edge_path_index_A << std::endl;
            std::cout << current_edge_path_index_B << std::endl;
        }

        assert(current_edge_index_A == current_edge_index_B);
        assert(current_edge_path_index_A == current_edge_path_index_B);
    }
}
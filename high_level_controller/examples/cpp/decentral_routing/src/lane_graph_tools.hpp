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

/**
 * \class LaneGraphTools
 * \brief Tools to deal with movement and collisions in the CPM Lab, modelled as a graph
 * \ingroup decentral_routing
 *
 * Which graph is used depends on which lane_graph.hpp is included by CMake
 */
class LaneGraphTools : public LaneGraph
{
    //! "Matrix" containing distances between edges
    std::vector<std::vector<double>> edges_s;
public:
    //! "3D-matrix" containing info about which points on the graph are collisions
    std::vector< std::vector< std::vector< std::vector<bool> > > > edge_path_collisions;
    //! Constructor for class LaneGraphTools
    LaneGraphTools();

    /**
     * \brief Converts from xy-coordinates to a position on the graph
     * \param pose Pose2D message object, containing x-y-coordinate and rotation
     * \param out_edge_index pass-by-reference for the corresponding edge; will be modified by function
     * \param out_edge_path_index pass-by-reference for the corresponding edge path; will be modified by function
     */
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index) const;

    /**
     * \brief Find edges that are "after" the given edge in the graph
     * \param edge_index Index of edge to find connected edges to
     */
    vector<size_t> find_subsequent_edges(int edge_index) const;

    /**
     * \brief Returns the position on the graph after moving a certain distance
     * \param route_edge_indices List of edge indices the vehicle will pass next
     * \param edge_index pass-by-reference for the corresponding edge; will be modified by function
     * \param edge_path_index pass-by-reference for the corresponding edge path; will be modified by function
     * \param delta_s pass-by-reference for distance to move; will be modified by function and return the distance moved beyond the last edge path
     *
     * Given a combination of a edge, an edge path and a distance delta s this will move along the path given by route_edge_indices,
     * until the distance delta_s isn't enough to advance to the next edge path.
     * The value of edge_index, edge_path_index and delta_s will be the resulting position,
     * with delta_s being the "left-over" distance
     */
    void move_along_route(vector<size_t> route_edge_indices, size_t &edge_index, size_t &edge_path_index, double &delta_s) const;
};

/**
 * \brief TODO
 * \ingroup decentral_routing
 */
extern const LaneGraphTools laneGraphTools;
extern const LaneGraphTools laneGraphTools;

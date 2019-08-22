#pragma once
#include "lane_graph.hpp"
#include "Pose2D.hpp"
#include <vector>
using std::vector;

class LaneGraphTools : public LaneGraph
{
    std::vector<std::vector<double>> edges_s;
public:
    LaneGraphTools();
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index);
    vector<size_t> find_subsequent_edges(int edge_index);
    void move_along_route(vector<size_t> route_edge_indices, size_t &edge_index, size_t &edge_path_index, double delta_s);
};

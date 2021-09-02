#pragma once
#include "lane_graph.hpp"
#include "Pose2D.hpp"
#include <vector>
using std::vector;

/**
 * \class LaneGraphTools
 * \brief TODO
 * \ingroup central_routing
 */
class LaneGraphTools : public LaneGraph
{
    //! TODO
    std::vector<std::vector<double>> edges_s;
public:
    //! TODO
    std::vector< std::vector< std::vector< std::vector<bool> > > > edge_path_collisions;

    //! Constructor TODO
    LaneGraphTools();

    /**
     * \brief TODO
     * \param pose TODO
     * \param out_edge_index TODO
     * \param out_edge_path_index TODO
     */
    bool map_match_pose(Pose2D pose, int &out_edge_index, int &out_edge_path_index) const;

    /**
     * \brief TODO
     * \param edge_index TODO
     */
    vector<size_t> find_subsequent_edges(int edge_index) const;

    /**
     * \brief TODO
     * \param route_edge_indices
     * \param edge_index
     * \param edge_path_index
     * \param delta_s
     */
    void move_along_route(vector<size_t> route_edge_indices, size_t &edge_index, size_t &edge_path_index, double &delta_s) const;
};

/**
 * \brief TODO
 * \ingroup central_routing
 */
extern const LaneGraphTools laneGraphTools;
#pragma once

#include <atomic>

/**
 * \struct CommonroadDrawConfiguration
 * \brief This class is not part of the commonroad specs. It defines a data structure that holds information on which
 * parts of the scenario should be visible to the user. The configuration can be set elsewhere and is applied here,
 * in the scenario, when drawing. Some "sub"-classes of the scenario, like lanelet, might have direct access to this
 * file, for easier data sharing
 * We use atomic bool because access from different parts of the program is allowed and desired
 * \ingroup lcc_commonroad
 */
struct CommonroadDrawConfiguration
{
    //! Sets if traffic signs should be drawn in the LCC Map View
    std::atomic_bool draw_traffic_signs{false};
    //! Sets if traffic lights should be drawn in the LCC Map View
    std::atomic_bool draw_traffic_lights{false};
    //! Sets if lanelet ID should be drawn in each lanelet segment in the LCC Map View
    std::atomic_bool draw_lanelet_id{false};
    //! Sets if the lanelet orientation in form of grey arrows should be drawn in the LCC Map View
    std::atomic_bool draw_lanelet_orientation{false};
    //! Sets if goal descriptions should be drawn on the goal states in the LCC Map View
    std::atomic_bool draw_goal_description{false};
    //! Sets if initial state descriptions should be drawn on the goal states in the LCC Map View
    std::atomic_bool draw_init_state{false};
    //! Sets if obstacle descriptions should be drawn on the obstacles in the LCC Map View
    std::atomic_bool draw_obstacle_description{false};
    //! Set and read current zoom factor in map view to re-scale text properly depending on the zoom level
    std::atomic<double> zoom_factor{100};
};
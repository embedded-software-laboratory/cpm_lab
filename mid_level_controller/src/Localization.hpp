#pragma once

#include "VehicleState.hpp"
#include "VehicleObservation.hpp"
#include <cassert>

/**
 * \struct LocalizationState
 * \brief TODO
 * \ingroup vehicle
 */
struct LocalizationState
{
    //! TODO
    uint64_t t = 0;
    //! TODO, local sensor data
    double imu_yaw = 0;
    //! TODO, local sensor data
    double odometer_distance = 0;
    //! TODO
    bool has_valid_observation = false;
    //! TODO, remote IPS data
    VehicleObservation vehicleObservation;
    //! Best estimate of the pose
    Pose2D pose = Pose2D(0,0,0);
};

#define LOCALIZATION_BUFFER_SIZE (512)

/**
 * \class Localization
 * \brief TODO
 * \ingroup vehicle
 */
class Localization
{
    /**
     * \brief The Localization saves a history of its state,
     * so the filter update can be recomputed when delayed observations arrive.
     */
    LocalizationState state_buffer[LOCALIZATION_BUFFER_SIZE];
    //! Index of oldest element, index of next overwrite
    size_t state_buffer_index = 0;

    /**
     * \brief TODO
     * \param i TODO
     */
    LocalizationState& get_state(size_t i)
    {
        assert(i < LOCALIZATION_BUFFER_SIZE);
        return state_buffer[(i + state_buffer_index) % LOCALIZATION_BUFFER_SIZE];
    }

    /**
     * \brief TODO
     * \param state TODO
     */
    void write_next_state(const LocalizationState& state)
    {
        state_buffer[state_buffer_index] = state;
        state_buffer_index = (state_buffer_index+1) % LOCALIZATION_BUFFER_SIZE;
    }

public:

    /**
     * \brief Assumptions:
     * The 'vehicleState' always has new sensor data.
     * The 'sample_vehicleObservation' update may be delayed and intermittent.
     * If no new observation is available, a repeat of the most recent one is expected.
     * \param t_now TODO
     * \param period TODO
     * \param vehicleState TODO
     * \param sample_vehicleObservation TODO
     * \param sample_vehicleObservation_age TODO
     */
    Pose2D update(
        uint64_t t_now,
        uint64_t period,
        VehicleState vehicleState,
        VehicleObservation sample_vehicleObservation,
        uint64_t sample_vehicleObservation_age
    );

    /**
     * \brief TODO
     */
    void reset();
    
};
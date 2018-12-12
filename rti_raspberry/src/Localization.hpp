#pragma once

#include "VehicleState.hpp"
#include "VehicleObservation.hpp"
#include <cassert>

struct LocalizationState
{
    uint64_t t = 0;
    double imu_yaw = 0; // local sensor data
    double odometer_distance = 0; // local sensor data
    bool has_valid_observation = false;
    VehicleObservation vehicleObservation; // remote IPS data
    Pose2D pose = Pose2D(0,0,0); // best estimate of the pose
};

#define LOCALIZATION_BUFFER_SIZE (512)

class Localization
{
    /*
     * The Localization saves a history of its state,
     * so the filter update can be recomputed when delayed observations arrive.
     */
    LocalizationState state_buffer[LOCALIZATION_BUFFER_SIZE];
    size_t state_buffer_index = 0; // index of oldest element, index of next overwrite

    LocalizationState& get_state(size_t i)
    {
        assert(i < LOCALIZATION_BUFFER_SIZE);
        return state_buffer[(i + state_buffer_index) % LOCALIZATION_BUFFER_SIZE];
    }

    void write_next_state(const LocalizationState& state)
    {
        state_buffer[state_buffer_index] = state;
        state_buffer_index = (state_buffer_index+1) % LOCALIZATION_BUFFER_SIZE;
    }

public:

    /* Assumptions:
     * The 'vehicleState' always has new sensor data.
     * The 'sample_vehicleObservation' update may be delayed and intermittent.
     * If no new observation is available, a repeat of the most recent one is expected.
     */
    Pose2D update(
        uint64_t t_now,
        VehicleState vehicleState,
        VehicleObservation sample_vehicleObservation,
        uint64_t sample_vehicleObservation_age
    );
    
};
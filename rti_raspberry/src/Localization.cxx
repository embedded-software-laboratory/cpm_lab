#include "Localization.hpp"
#include <cmath>

void filter_update_step(const LocalizationState& previous, LocalizationState& current)
{
    double delta_s = current.odometer_distance - previous.odometer_distance;
    double delta_yaw = remainder(current.imu_yaw - previous.imu_yaw, 2*M_PI);

    // ignore signal discontinuities
    if(!(-0.5 < delta_s && delta_s < 0.5)) {
        delta_s = 0;
    }

    if(!(-1.5 < delta_yaw && delta_yaw < 1.5)) {
        delta_yaw = 0;
    }

    // Dead reckoning update
    Pose2D new_pose = previous.pose;
    new_pose.yaw(new_pose.yaw() + delta_yaw);
    new_pose.x(new_pose.x() + delta_s * cos(new_pose.yaw()));
    new_pose.y(new_pose.y() + delta_s * sin(new_pose.yaw()));


    if(current.has_valid_observation)
    {
        // TODO IPS update
    }

    current.pose = new_pose;
}


Pose2D Localization::update(
    uint64_t t_now,
    VehicleState vehicleState,
    VehicleObservation sample_vehicleObservation,
    uint64_t sample_vehicleObservation_age
)
{

    // Save new sensor data, update current step
    {
        LocalizationState localizationStateNew;
        localizationStateNew.t = t_now;
        localizationStateNew.imu_yaw = vehicleState.pose().yaw();
        localizationStateNew.odometer_distance = vehicleState.odometer_distance();
        write_next_state(localizationStateNew);

        filter_update_step(get_state(LOCALIZATION_BUFFER_SIZE-2), get_state(LOCALIZATION_BUFFER_SIZE-1));
    }

    // Check for new observation. Reprocess if necessary
    if(sample_vehicleObservation_age < 10000000000ull)
    {
        int reprocessing_start_index = -1;

        // search for state index corresponding to the given observation
        for (int i = LOCALIZATION_BUFFER_SIZE-1; i > 0; i--)
        {
            LocalizationState& state_i = get_state(i);

            // found the corresponding state
            if(state_i.t == sample_vehicleObservation.header().create_stamp().nanoseconds())
            {
                // have we already done this on a previous update()? then skip it
                if(state_i.has_valid_observation) break;

                state_i.has_valid_observation = true;
                state_i.vehicleObservation = sample_vehicleObservation;

                // we have a new observation, need to do reprocessing
                reprocessing_start_index = i;
                break;
            }
        }

        // reprocessing
        if(reprocessing_start_index > 0)
        {
            for (int i = reprocessing_start_index; i < LOCALIZATION_BUFFER_SIZE; ++i)
            {
                filter_update_step(get_state(i-1), get_state(i));
            }
        }
    }


    // output latest pose
    return get_state(LOCALIZATION_BUFFER_SIZE-1).pose;
}
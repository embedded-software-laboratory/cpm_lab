#include "MultiVehicleTrajectoryPlanner.hpp"



MultiVehicleTrajectoryPlanner::MultiVehicleTrajectoryPlanner(uint64_t dt_nanos):dt_nanos(dt_nanos){}

std::vector<VehicleCommandTrajectory> MultiVehicleTrajectoryPlanner::get_trajectory_commands(uint64_t t_now)
{
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<VehicleCommandTrajectory> result;
    for(auto &e:trajectory_point_buffer)
    {
        VehicleCommandTrajectory vehicleCommandTrajectory;
        vehicleCommandTrajectory.vehicle_id(e.first);
        vehicleCommandTrajectory.trajectory_points(rti::core::vector<TrajectoryPoint>(e.second));
        vehicleCommandTrajectory.header().create_stamp().nanoseconds(t_now); //You just need to set t_now here, as it was created at t_now
        vehicleCommandTrajectory.header().valid_after_stamp().nanoseconds(t_now + 2000000000ull); //Hardcoded value from the planner (t_start), should be correct (this value should correlate with the trajectory point that should be valid at t_now)
        result.push_back(vehicleCommandTrajectory);
    }
    return result;
}

void MultiVehicleTrajectoryPlanner::set_real_time(uint64_t t)
{
    std::lock_guard<std::mutex> lock(mutex);
    t_real_time = t;
}


void MultiVehicleTrajectoryPlanner::add_vehicle(std::shared_ptr<VehicleTrajectoryPlanningState> vehicle)
{   //fill vector with future trajectory points of other vehicles
    assert(!started);
    trajectoryPlans[vehicle->get_vehicle_id()] = vehicle;
}

void MultiVehicleTrajectoryPlanner::start()
{
    assert(!started);
    started = true;

    planning_thread = std::thread([this](){
        uint64_t t_planning = 0;

        {
            std::lock_guard<std::mutex> lock(mutex); 
            t_planning = t_real_time;
        }

        while(1)
        {
            // Priority based collision avoidance: Every vehicle avoids 
            // the 'previous' vehicles, in this example those with a smaller ID.
            vector< std::shared_ptr<VehicleTrajectoryPlanningState> > previous_vehicles;
            
            for(auto &e:trajectoryPlans)
            {
                e.second->avoid_collisions(previous_vehicles);
                previous_vehicles.push_back(e.second);
            }

            {
                std::lock_guard<std::mutex> lock(mutex); 

                if(t_start == 0)
                {
                    t_start = t_real_time + 2000000000ull;
                }

                for(auto &e:trajectoryPlans)
                {
                    while(trajectory_point_buffer[e.first].size() > 50)
                    {
                        trajectory_point_buffer[e.first].erase(trajectory_point_buffer[e.first].begin());
                    }
                    auto trajectory_point = e.second->get_trajectory_point();
                    trajectory_point.t().nanoseconds(trajectory_point.t().nanoseconds() + t_start);
                    trajectory_point_buffer[e.first].push_back(trajectory_point);
                }
            }

            for(auto &e:trajectoryPlans)
            {
                e.second->apply_timestep(dt_nanos);
            }

            t_planning += dt_nanos;

            while(t_real_time + 6000000000ull < t_planning) usleep(110000);
        }
    });

}
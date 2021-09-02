#pragma once
#include "VehicleObservation.hpp"
#include "cpm/Writer.hpp"
#include <list>
#include <string>

/**
 * \class SimulationIPS
 * \brief TODO
 * \ingroup vehicle
 */
class SimulationIPS
{
    //! TODO
    cpm::Writer<VehicleObservation> writer_vehicleObservation;

    //! TODO
    std::list<VehicleObservation> delay_buffer;

public:
    /**
     * \brief TODO Constructor
     * \param topic_name TODO
     */
    SimulationIPS(std::string topic_name);

    /**
     * \brief TODO
     * \param simulatedState TODO
     */
    void update(VehicleObservation simulatedState);
    
};
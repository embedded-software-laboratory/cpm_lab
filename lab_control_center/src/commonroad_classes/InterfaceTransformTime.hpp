#pragma once

/**
 * \class InterfaceTransformTime
 * \brief This interface requires the deriving classes to implement a transform function for timing
 * \ingroup lcc_commonroad
 */
class InterfaceTransformTime
{
public:
    /**
     * \brief This function is used to change timing-related values, like velocity, where needed
     * \param time_scale The factor with which time step size was changed (e.g. 1.0->2.0 results in a factor of 0.5)
     */
    virtual void transform_timing(double time_scale) = 0;

    //! Destructor. Good practice
    virtual ~InterfaceTransformTime() {};
};
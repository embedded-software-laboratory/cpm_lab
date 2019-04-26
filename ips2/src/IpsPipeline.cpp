#include "IpsPipeline.hpp"


IpsPipeline::IpsPipeline()
{

}


void IpsPipeline::apply(LedPoints led_points)
{

    std::cout << "recvd " << led_points.led_points().size() << " LEDs" << std::endl;

    VehiclePointTimeseries asd;

}

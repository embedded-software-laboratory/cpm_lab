#include <libxml++-2.6/libxml++/libxml++.h>

#include <iostream>
#include <vector>

#include "commonroad_classes/CommonRoadScenario.hpp"

int main()
{
    std::string filepath = "/home/cpm-lab/dev/software/LabControlCenter/ui/map_view/C-USA_US101-30_1_T-1.xml";

    CommonRoadScenario commonroad_scenario(filepath);

    return 0;
}
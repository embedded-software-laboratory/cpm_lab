#include <libxml++-2.6/libxml++/libxml++.h>

#include <iostream>
#include <vector>

#include "commonroad_classes/CommonRoadScenario.hpp"

int main()
{
    std::string filepath_2018 = "/home/cpm-lab/dev/software/lab_control_center/test/C-USA_US101-30_1_T-1.xml";
    std::string filepath_2020 = "/home/cpm-lab/dev/software/lab_control_center/test/documentation_XML_commonRoad_minimalExample_2020a.xml";

    CommonRoadScenario commonroad_scenario_2018;
    commonroad_scenario_2018.load_file(filepath_2018);
    CommonRoadScenario commonroad_scenario_2020;
    commonroad_scenario_2020.load_file(filepath_2020);

    return 0;
}
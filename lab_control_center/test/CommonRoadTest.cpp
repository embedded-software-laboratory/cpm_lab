// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
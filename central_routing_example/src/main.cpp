
#include "lane_graph.hpp"
#include "cpm/Logging.hpp"
#include "cpm/CommandLineReader.hpp"
#include "cpm/init.hpp"
#include <iostream>



int main(int argc, char *argv[])
{
    cpm::init(argc, argv);
    cpm::Logging::Instance().set_id("central_routing_example");
    const bool enable_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    LaneGraph laneGraph;
    std::cout << laneGraph.n_nodes << std::endl;
}
/**
 * \class main.cpp
 * \brief This class exists for test purposes only
 */

#include "cpm/Parameter.hpp"
#include "ParameterServer.hpp"

#include <dds/pub/ddspub.hpp>

int main(int argc, char *argv[])
{
    ParameterStorage storage("../test.yaml");
    ParameterServer server(storage);

    std::cout << "Parameter server started" << std::endl;

    //Stop the program when enter is pressed
    std::cin.get();
    std::cout << "Exiting program" << std::endl;

    //server.store_configuration();

    return 0;
}
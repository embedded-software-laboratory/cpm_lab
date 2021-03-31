#include "../include/go_to_formation.hpp"

int main(int argc, char *argv[])
{

    std::cout << "Starting go to formation" << std::endl;

    int returned = go_to_formation(argc, argv);


    std::cout << "Go to formation finished with: " << returned << std::endl;
}

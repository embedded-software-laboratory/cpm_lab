#include <stdlib.h>
#include <iostream>
#include <string>
#include "labcam/LabCam.hpp"

//Suppress warning for unused parameter of main
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

int main(int argc, char *argv[])
{
    LabCam labcam;

    labcam.startRecording(".", "awesome_recording1");

    std::cout << "." << std::endl;
    std::cin.get();

    std::cout << "stopping lab cam" << std::endl;
    labcam.stopRecording();
    std::cout << "Press enter to start recording again" << std::endl;

    
    std::cin.get();
    labcam.startRecording(".", "awesome_recording2");


    std::cin.get();
    labcam.stopRecording();

    while(1);
}

#pragma GCC diagnostic pop

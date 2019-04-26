#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char* argv[])
{
    std::thread rti_replay_thread([](){system("rtireplay -cfgName mydefault -cfgFile recordings/replay_config.xml");});
    
    while(1)
    {
        std::cout << "." << std::endl;
        usleep(500000);
    }
    return 0;
}
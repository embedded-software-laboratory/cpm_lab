#include "defaults.hpp"
#include "Joystick.hpp"

#include <unistd.h>

int main(/*int argc, char *argv[]*/)
{
    auto joystick = make_shared<Joystick>("/dev/input/js1");

    while(1) {
        cout << joystick->getAxis(1) << endl;
        usleep(50000);
    }

    return 0;
}
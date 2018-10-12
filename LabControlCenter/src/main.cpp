#include "defaults.hpp"
#include "Joystick.hpp"


int main(/*int argc, char *argv[]*/)
{
    auto joystick = make_shared<Joystick>("/dev/input/js1");
    return 0;
}
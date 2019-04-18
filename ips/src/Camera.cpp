#include "Camera.h"


Camera::Camera(const std::string &serial_number):
	serial_number(serial_number)
{
}


Camera::~Camera()
{
}

std::string Camera::getSerialNumber() { return serial_number; }

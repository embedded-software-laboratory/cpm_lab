#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbCamera.h>
#include <opencv2/opencv.hpp>

#include "default.h"

class Camera
{

public:
	Camera(const std::string &serial_number);
	~Camera();

	/**
		returns serial number of the camera
	*/
	std::string getSerialNumber();

	/**
		grabs one image from the camera
	*/
	virtual bool grabImage(std::shared_ptr<TimedImage> &image) = 0;

	/**
		closes the camera
	*/
	virtual void close() = 0;

private:
	/**
		serial number of the camera
	*/
	std::string serial_number;

};

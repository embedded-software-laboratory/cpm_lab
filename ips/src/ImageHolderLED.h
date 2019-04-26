#pragma once

#include "DetectedObjects.h"

#include "CameraStream.h"
#include "CameraFileWrapper.h"

class ImageHolderLED
{
public:
	ImageHolderLED(const std::string &serial_no);
	ImageHolderLED(const std::string &serial_no, const std::string &filename);
	~ImageHolderLED();

	/*
		close the camera
	*/
	void close();

	/*
		detect points in image
	*/
	void detectObjectsInCurrentImages(std::shared_ptr<DetectedObjects> &detectedObjects);

private:

	/*
		wether we use the cameraFileWrapper or the real camera
	*/
	bool simulation;

	/*
		wrapper for pylon camera // or camera File Wrapper
	*/
	std::shared_ptr<Camera> camera;

	/*
		get all Points/Blobs in the specified image
	*/
	TimedPoints getPointsInImage(const TimedImage &image);
};


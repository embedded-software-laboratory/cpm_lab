#pragma once

#include "ImageProcessingHelper.h"
#include "default.h"
#include "Camera.h"
#include "ThreadSafeQueue.h"

class CameraFileWrapper
	: public Camera
{
public:
	CameraFileWrapper(const std::string &serial_no, const std::string &filename);
	~CameraFileWrapper();

	/**
		grab one image
	*/
	bool grabImage(std::shared_ptr<TimedImage> &image);

	/**
		close camera
	*/
	void close();

private:
	/**
		saves all points of each image
	*/
	std::vector<std::vector <ImagePoint>> pointsPerImageFromFile;
	/**
		saves all timestamps corresponding to the points
	*/
	std::vector<Time_Stamp> timestamps;

	/**
		id of the image for the queue
	*/
	int id = 0;

	/**
		counter of image which is next received
	*/
	size_t currentImage = 0;

	/**
		thread for grabbing the images
		write all 20ms an image to the queue
	*/
	std::thread grabbingThread;

	/**
		queue of the images grabbed
	*/
	ThreadSafeQueue<TimedImage> images;

	/**
		indicates whehter grabbing should be continued
	*/
	bool continue_grabbing = true;

	/**
		get points of the next image
	*/
	void getPointsInImages(std::vector<ImagePoint> &pointsPerImage, Time_Stamp &timestamp);

	/**
		draw points into an image
	*/
	void drawPoints(const cv::Mat &image, const std::vector<ImagePoint> &points);

	/**
		function for thread that grabs images
	*/
	void grabbingImages();
	
};


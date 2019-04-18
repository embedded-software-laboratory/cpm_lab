#pragma once

#include <thread>
#include <mutex>

#include "Camera.h"
#include "GlobalDataHelper.h"
#include "ThreadSafeQueue.h"

class CameraStream :
	public Camera
{
public:
	CameraStream(const std::string &serial_number);
	~CameraStream();

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
		id of the image to track the order of the images
	*/
	int id = 0;

	/**
		if continue_grabbing is false the grabbing thread is joined
	*/
	bool continue_grabbing = true;

	/**
		number of buffers for the stream grabber
	*/
	static const int numBuffers = 10;

	/**
		thread which grabs the images
	*/
	std::thread grabbingThread;

	/**
		queue in which the grabbed images are stored
	*/
	ThreadSafeQueue<std::pair<int, Pylon::GrabResult>> imageQueue;



	/**
		function for the thread
	*/
	void grabbingImages();

	/**
		add grabbed image to queue
	*/
	void addToQueue(Pylon::GrabResult &result);
};


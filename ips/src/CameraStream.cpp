#include "CameraStream.h"

#include <pylon/usb/BaslerUsbInstantCamera.h>
using namespace Pylon;
using namespace Basler_UsbCameraParams;

CameraStream::CameraStream(const std::string &serial_number)
	: Camera(serial_number)
{
	grabbingThread = std::thread([this] { this->grabbingImages(); });
}


CameraStream::~CameraStream()
{
}

void CameraStream::close() {
	continue_grabbing = false;
	grabbingThread.join();
}


bool CameraStream::grabImage(std::shared_ptr<TimedImage> &image) {

	if (GlobalDataHelper::getInstance().isCanceled()) {
		return false;
	}

	std::pair<int, Pylon::GrabResult> result;
	imageQueue.pop(result);

	Time_Stamp timestamp = GlobalDataHelper::getInstance().getStartTime() + result.second.GetTimeStamp();
	int rows = result.second.GetSizeY();
	int cols = result.second.GetSizeX();
	void *data = result.second.Buffer();
	cv::Mat img = cv::Mat(rows, cols, CV_8UC1, data);
	image = std::make_shared<TimedImage>(result.first, img.clone(), timestamp);

	return true;
}


void CameraStream::addToQueue(Pylon::GrabResult &result) {
	imageQueue.push(std::pair<int, Pylon::GrabResult>(id, result));
	id++;
}

//function for thread
void CameraStream::grabbingImages() {
	Pylon::PylonAutoInitTerm autoInitTerm;


	try
	{
		// Enumerate GigE cameras
		Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
		Pylon::ITransportLayer *pTl = TlFactory.CreateTl(Pylon::CBaslerUsbCamera::DeviceClass());
		Pylon::DeviceInfoList_t devices;
		if (0 == pTl->EnumerateDevices(devices)) {
			std::cerr << "No camera present!" << std:: endl;
			GlobalDataHelper::getInstance().cancelProgram();
			return;
		}

		// Create a camera object
		Pylon::CBaslerUsbCamera camera(pTl->CreateDevice(devices[0]));

		// Open the camera object
		camera.Open();

		// Parameterize the camera

		// Mono8 pixel format
		camera.PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_Mono8);

		// Maximized AOI
		camera.OffsetX.SetValue(0);
		camera.OffsetY.SetValue(0);
		camera.Width.SetValue(camera.Width.GetMax());
		camera.Height.SetValue(camera.Height.GetMax());


        // Set lowest gain, results in lowest image noise
        camera.GainAuto.SetValue(GainAuto_Off);
        camera.Gain.SetValue(camera.Gain.GetMin());

		// Continuous mode, no external trigger used
		camera.TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
		camera.TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_Off);
		camera.AcquisitionMode.SetValue(Basler_UsbCameraParams::AcquisitionMode_Continuous);

		// Configure exposure time and mode
		camera.ExposureMode.SetValue(Basler_UsbCameraParams::ExposureMode_Timed);
		camera.ExposureAuto.FromString("Off");
		camera.ExposureTime.SetValue(GlobalDataHelper::getInstance().getGlobalConst().EXPOSURE_TIME);

        // Set fixed FPS
        camera.AcquisitionFrameRateEnable.SetValue(true);
        camera.AcquisitionFrameRate.SetValue(50);
        camera.DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitMode_Off);

		
		// check whether stream grabbers are avalaible
		if (camera.GetNumStreamGrabberChannels() == 0) {
			std::cerr << "Camera doesn't support stream grabbers." << std::endl;
		}
		else {
			// Get and open a stream grabber
			//Pylon::IStreamGrabber* pGrabber = camera.GetStreamGrabber(0);
			Pylon::CBaslerUsbCamera::StreamGrabber_t StreamGrabber(camera.GetStreamGrabber(0));
			StreamGrabber.Open();

			// Parameterize the stream grabber
			const int bufferSize = (int)camera.PayloadSize();
			StreamGrabber.MaxBufferSize = bufferSize;
			StreamGrabber.MaxNumBuffer = numBuffers;
			StreamGrabber.PrepareGrab();

			// Allocate and register image buffers, put them into the
			// grabber's input queue
			unsigned char* ppBuffers[numBuffers];
			Pylon::StreamBufferHandle handles[numBuffers];
			for (int i = 0; i < numBuffers; ++i)
			{
				ppBuffers[i] = new unsigned char[bufferSize];
				handles[i] = StreamGrabber.RegisterBuffer(ppBuffers[i], bufferSize);
				StreamGrabber.QueueBuffer(handles[i]);
			}

			//save current time
            struct timespec t;
			camera.TimestampLatch();

            clock_gettime(CLOCK_REALTIME, &t);
            uint64_t startTime = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
            int64_t startTicks = camera.TimestampLatchValue.GetValue();

			GlobalDataHelper::getInstance().setStartTime(startTime, startTicks);


			// Start image acquisition
			camera.AcquisitionStart.Execute();

			Pylon::GrabResult Result;
			while(continue_grabbing) {
				// Wait for the grabbed image with a timeout of 3 seconds
				if (StreamGrabber.GetWaitObject().Wait(3000)) {
					// Get an item from the grabber's output queue
					if (!StreamGrabber.RetrieveResult(Result)) {
						std::cerr << "Failed to retrieve an item from the output queue" << std::endl;
					}
					if (Result.Succeeded()) {
						// Grabbing was successful. Process the image.
						addToQueue(Result);
					}
					else {
						std::cerr << "Grab failed: " << Result.GetErrorDescription() << std::endl;
					}
					// Requeue the buffer
					StreamGrabber.QueueBuffer(Result.Handle(), Result.Context());
				}
				else {
					std::cerr << "timeout occurred when waiting for a grabbed image" << std::endl;
				}
			}

			// Finished. Stop grabbing and do clean-up

			// The camera is in continuous mode, stop image acquisition
			camera.AcquisitionStop.Execute();

			// Flush the input queue, grabbing may have failed
			StreamGrabber.CancelGrab();

			// Consume all items from the output queue
			while (StreamGrabber.GetWaitObject().Wait(0)) {
				StreamGrabber.RetrieveResult(Result);
				if (Result.Status() == Pylon::Canceled)
					std::cout << "Got canceled buffer" << std::endl;
			}

			// Deregister and free buffers
			for (int i = 0; i < numBuffers; ++i) {
				StreamGrabber.DeregisterBuffer(handles[i]);
				delete[] ppBuffers[i];
			}

			// Clean up
			StreamGrabber.FinishGrab();
			StreamGrabber.Close();
		}

		camera.Close();
		TlFactory.ReleaseTl(pTl);
	}
	catch (Pylon::GenericException &e)
	{
		// Error handling
		std::cerr << "An exception occurred!" << std::endl << e.GetDescription() << std::endl;
		GlobalDataHelper::getInstance().cancelProgram();
	}
}
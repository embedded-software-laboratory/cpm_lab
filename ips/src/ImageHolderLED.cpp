#include "ImageHolderLED.h"


ImageHolderLED::ImageHolderLED(const std::string &serial_no)
{
	camera = std::make_shared<CameraStream>(serial_no);
	simulation = false;
}


ImageHolderLED::ImageHolderLED(const std::string &serial_no, const std::string &filename) {
	camera = std::make_shared<CameraFileWrapper>(serial_no, filename);
	simulation = true;
}


ImageHolderLED::~ImageHolderLED()
{
}


void ImageHolderLED::close() {
	camera->close();
}

void ImageHolderLED::detectObjectsInCurrentImages(std::shared_ptr<DetectedObjects> &detectedObjects) {

	detectedObjects = std::make_shared<DetectedObjects>();

	std::shared_ptr<TimedImage> image;
	bool success = camera->grabImage(image);

	if (!success) {
		return;
	}

	struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

	detectedObjects->setID(image->id);

	TimedPoints points = getPointsInImage(*image);
	detectedObjects->addPointsForImage(points);

	image->image.release();

    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
	GlobalDataHelper::getInstance().addDurationImageHolder(t2 - t1);
}

//we want nearest integer to moments -> so conversion neccessary
TimedPoints ImageHolderLED::getPointsInImage(const TimedImage &image) {
	double min_thresh = 100;
	double max_thresh = 255;

	cv::Mat img;
	cv::threshold(image.image, img, min_thresh, max_thresh, cv::THRESH_BINARY);

	std::vector<ImagePoint> points, undistortedPoints;
	std::vector<std::vector<cv::Point> > contours;

	cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

	for (std::vector<cv::Point> contour : contours) {

		double size = cv::contourArea(contour);

		if (size < 15 && size > 6) {
			cv::Moments M = cv::moments(contour);
			ImagePoint point((M.m10 / (M.m00 + 1e-5)), (M.m01 / (M.m00 + 1e-5)));
			points.push_back(point);
		}
	}

	std::cout << "Found " << points.size() << " LEDs in image" << std::endl;

	if (simulation) {
		undistortedPoints = points;
	}
	else {
		GlobalDataHelper::getInstance().getCameraParameters()->undistortPoints(points, undistortedPoints);
	}

	return TimedPoints(undistortedPoints, image.timestamp);
}

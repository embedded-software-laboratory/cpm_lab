#include "IllustrationHelper.h"


void IllustrationHelper::drawPoints(const std::vector<ImagePoint> &points) {

	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	for (ImagePoint point : points) {
		cv::circle(img_rgb, point, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
	}

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawPoints(const std::vector<ImagePoint> &points, const ImagePoint &startPoint) {

	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	for (ImagePoint point : points) {
		cv::circle(img_rgb, point, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
	}

	cv::circle(img_rgb, startPoint, 4, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawPoints(const std::vector<ImagePoint> &pointsPos, const std::vector<ImagePoint> &pointsId) {

	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	for (ImagePoint point : pointsPos) {
		cv::circle(img_rgb, point, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
	}

	for (ImagePoint point : pointsId) {
		cv::circle(img_rgb, point, 4, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);
	}

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawPoints(const std::vector<ImagePoint> &pointsPos, const std::vector<ImagePoint> &pointsId, const ImagePoint &center) {
	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	for (ImagePoint point : pointsPos) {
		cv::circle(img_rgb, point, 4, cv::Scalar(250, 118, 242), cv::FILLED, 8, 0);
	}

	for (ImagePoint point : pointsId) {
		cv::circle(img_rgb, point, 3, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);
	}

	cv::circle(img_rgb, center, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawPoints(const std::vector<cv::Point> &points) {

	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	for (ImagePoint point : points) {
		cv::circle(img_rgb, point, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
	}

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawPoints(const std::vector<MatchedPoints> &points) {

	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);

	int i = 0;
	for (MatchedPoints matched : points) {
		//cv::Scalar color = cv::Scalar((255 % (i + 17)) * 7, 255 - (17 * i),  255 - (i * 102367) % 255);
		cv::Scalar color = cv::Scalar(0, 255 * (i % 2), 255 * ((i + 1) % 2));
		for (ImagePoint point : matched.points) {
			cv::circle(img_rgb, point, 4, color, cv::FILLED, 8, 0);
			cv::imshow("Circles", img_rgb);

			cv::waitKey(100);
		}
		i++;
	}

	//cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawOptions(const std::vector<ImagePoint> &bestOption, const std::vector<std::vector<ImagePoint>> &options) {
	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);

	for (std::vector<ImagePoint> option : options) {
		drawOption(option, cv::Scalar(0, 0, 255), img_rgb);
	}

	drawOption(bestOption, cv::Scalar(255, 0, 0), img_rgb);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawOption(const std::vector<ImagePoint> &option, const cv::Scalar &color, const cv::Mat &image) {
	for (ImagePoint point : option) {
		cv::circle(image, point, 4, color, cv::FILLED, 8, 0);
	}
}

void IllustrationHelper::drawOption(const std::vector<ImagePoint> &option) {
	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);

	drawOption(option, cv::Scalar(0, 0, 255), img_rgb);

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawContour(const std::vector<ImagePoint> &points, const std::vector<cv::Point2f> &contour) {
	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	double x_min = DBL_MAX;
	double x_max = 0;
	double y_min = DBL_MAX;
	double y_max = 0;
	for (cv::Point2f point : contour) {
		if (point.x < x_min) {
			x_min = point.x;
		}
		if (point.x > x_max) {
			x_max = point.x;
		}
		if (point.y < y_min) {
			y_min = point.y;
		}
		if (point.y > y_max) {
			y_max = point.y;
		}
	}

	cv::Point2d p1(x_min, y_min);
	cv::Point2d p2(x_max, y_max);

	cv::Rect rect(p1, p2);

	cv::rectangle(img_rgb, rect, cv::Scalar(0, 255, 0), 4);


	for (ImagePoint point : points) {
		cv::circle(img_rgb, point, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
	}

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", img_rgb);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);

}


void IllustrationHelper::drawVehicleBacks(std::vector<std::pair<ImagePoint, ImagePoint>> &positions) {
	cv::Mat image = cv::Mat::zeros(2048, 2048, CV_8UC3);

	for (std::pair<ImagePoint, ImagePoint> position : positions) {
		cv::circle(image, position.first, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
		cv::circle(image, position.second, 4, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
	}

	cv::namedWindow("Circles", cv::WINDOW_NORMAL);
	cv::imshow("Circles", image);
	cv::resizeWindow("Circles", 500, 500);

	cv::waitKey(50000);
}

void IllustrationHelper::drawQRCodeCorners(const std::vector<ImagePoint> &points) {

	cv::Mat image = cv::imread("QRCodes/3VerschiedeneQ.png", cv::IMREAD_GRAYSCALE);

	// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
	cv::Mat img_rgb(image.size(), CV_8UC3);

	// convert grayscale to color image
	cv::cvtColor(image, img_rgb, cv::COLOR_GRAY2RGB);

	int i = 0;
	for (ImagePoint point : points) {
		cv::Scalar color(100 + i, 0 + i * 3, 255 - i);
		cv::circle(img_rgb, point, 8, color, cv::FILLED, 8, 0);
		i = i + 40;
	}

	cv::namedWindow("QRCodes", cv::WINDOW_NORMAL);
	cv::imshow("QRCodes", img_rgb);
	cv::resizeWindow("QRCodes", 500, 500);

	cv::waitKey(50000);
}


#pragma once
#include "default.h"

class IllustrationHelper
{
public:
	//singelton
	static IllustrationHelper& getInstance()
	{
		static IllustrationHelper instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	~IllustrationHelper() {}

	/**
		draw points in image
	*/
	void drawPoints(const std::vector<ImagePoint> &points);
	void drawPoints(const std::vector<cv::Point> &points);
	void drawPoints(const std::vector<MatchedPoints> &points);
	void drawPoints(const std::vector<ImagePoint> &points, const ImagePoint &startPoint);
	void drawPoints(const std::vector<ImagePoint> &pointsPos, const std::vector<ImagePoint> &pointsId);
	void drawPoints(const std::vector<ImagePoint> &pointsPos, const std::vector<ImagePoint> &pointsId, const ImagePoint &center);

	void drawOptions(const std::vector<ImagePoint> &bestOption, const std::vector<std::vector<ImagePoint>> &options);
	void drawOption(const std::vector<ImagePoint> &option, const cv::Scalar &color, const cv::Mat &image);
	void drawOption(const std::vector<ImagePoint> &option);

	void drawContour(const std::vector<ImagePoint> &points, const std::vector<cv::Point2f> &contour);
	void drawVehicleBacks(std::vector<std::pair<ImagePoint, ImagePoint>> &positions);

	void drawQRCodeCorners(const std::vector<ImagePoint> &points);

private:
	/**
		constrcutor only private
	*/
	IllustrationHelper() {}

	/**
		guarantees that no further instance can
		be constructed via copy-constructor
	*/
	IllustrationHelper(const IllustrationHelper&);

	/**
		avoids für instances with copy
	*/
	IllustrationHelper & operator = (const IllustrationHelper &);
};


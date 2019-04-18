#pragma once
#include "default.h"
#include "yaml-cpp/yaml.h"

class CameraParameter
{
public:
	CameraParameter();
	~CameraParameter();

	/**
		set camera parameters from yaml files
	*/
	void setExtrinsicParametersFromYAML(const std::string &filename);
	void setIntrinsicParametersFromYAML(const std::string &filename);

	/**
		call this function once after extrinsic and intrinsic parameters are set
	*/
	void initializeMatrices();

	/**
		undistort an image
	*/
	void undistortImage(const cv::Mat &distoredImage, cv::Mat &undistortedImage);

	/**
		undistort points
	*/
	void undistortPoints(const std::vector<ImagePoint> &distortedPoint, std::vector<ImagePoint> &undistortedPoint);

	/**
		project WorldPoint to undistored Image
	*/
	ImagePoint projectPoint(const WorldPoint &worldPoint);

	/**
		reproject undistored point to WorldPoint
	*/
	WorldPoint reprojectPoint(const ImagePoint &imagePoint, const double height);


private:

	/**
		intrinsic parameters
	*/

	double fx;
	double fy;
	double cx;
	double cy;
	double k1;
	double k2;
	double k3;
	double p1;
	double p2;


	cv::Matx33d cameraMatrix;
	cv::Vec<double, 5> distCoeffs;


	/**
		extrinsic parameters
	*/
	cv::Matx33d R;
	cv::Vec3d T;

	/**
		helper mat for reprojection
	*/
	cv::Matx33d R_inverted;
	cv::Matx33d cameraMatrix_inverted;
	cv::Matx33d A;
	cv::Vec3d B;
};


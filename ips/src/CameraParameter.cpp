#include "CameraParameter.h"


CameraParameter::CameraParameter()
{
}


CameraParameter::~CameraParameter()
{
}



void CameraParameter::setExtrinsicParametersFromYAML(const std::string &filename) {

	// Read camera parameters
	YAML::Node extrinsic_params = YAML::LoadFile(filename);

	cv::Matx33d R_;
	std::vector<double> R_vec = extrinsic_params["R"].as<std::vector<double>>();
	assert(R_vec.size() == 9);
	for (int i = 0; i < 9; ++i) R_.val[i] = R_vec[i];
	R = R_;

	cv::Vec3d T_;
	std::vector<double> T_vec = extrinsic_params["T"].as<std::vector<double>>();
	assert(T_vec.size() == 3);
	for (int i = 0; i < 3; ++i) T_.val[i] = T_vec[i];
	T = T_;
}


void CameraParameter::setIntrinsicParametersFromYAML(const std::string &filename) {

	// Read camera parameters
	YAML::Node intrinsic_params = YAML::LoadFile(filename);

	fx = intrinsic_params["fx"].as<double>();
	fy = intrinsic_params["fy"].as<double>();
	cx = intrinsic_params["cx"].as<double>();
	cy = intrinsic_params["cy"].as<double>();
	k1 = intrinsic_params["k1"].as<double>();
	k2 = intrinsic_params["k2"].as<double>();
	k3 = intrinsic_params["k3"].as<double>();
	p1 = intrinsic_params["p1"].as<double>();
	p2 = intrinsic_params["p2"].as<double>();
}

void CameraParameter::initializeMatrices() {
	cameraMatrix = cv::Matx33d(
		fx, 0, cx,
		0, fy, cy,
		0, 0, 1);

/*	cameraMatrix.ptr<float>(0)[0] = fx;
	cameraMatrix.ptr<float>(0)[2] = cx;
	cameraMatrix.ptr<float>(0)[0] = fx;
	cameraMatrix.ptr<float>(0)[2] = cx;*/

	distCoeffs = cv::Vec<double, 5>(k1, k2, p1, p2, k3);

	cv::invert(cameraMatrix, cameraMatrix_inverted);
	cv::invert(R, R_inverted);

	A = R_inverted * cameraMatrix_inverted;
	B = R_inverted * T;

}


void CameraParameter::undistortImage(const cv::Mat &distoredImage, cv::Mat &undistortedImage) {
	cv::undistort(distoredImage, undistortedImage, cv::Mat(cameraMatrix), cv::Mat(distCoeffs));
}


void CameraParameter::undistortPoints(const std::vector<ImagePoint> &distortedPoints, std::vector<ImagePoint> &undistortedPoints) {
	
	if (distortedPoints.empty()) {
		return;
	}


	std::vector<cv::Point2f> points, undisPoints;
	for (ImagePoint point : distortedPoints) {
		points.push_back(point);
	}
	
	cv::undistortPoints(points, undisPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

	for (cv::Point point : undisPoints) {
		undistortedPoints.push_back(point);
	}
}

ImagePoint CameraParameter::projectPoint(const WorldPoint &worldPoint) {

	cv::Matx34d RT;
	cv::hconcat(R, T, RT);


	cv::Vec4d wp(worldPoint.x, worldPoint.y, worldPoint.z, 1);
	cv::Vec3d ip = cameraMatrix * RT * wp;

	int x = (int)(ip(0) / ip(2));
	int y = (int)(ip(1) / ip(2));

	return ImagePoint(x, y);
}

WorldPoint CameraParameter::reprojectPoint(const ImagePoint &imagePoint, const double height) {
	
	cv::Vec3d ip(imagePoint.x, imagePoint.y, 1);

	cv::Vec3d m = A * ip;

	double k = (height + B(2)) / m(2);
	double U = k * m(0) - B(0);
	double V = k * m(1) - B(1);

	return WorldPoint(U, V, height);
}
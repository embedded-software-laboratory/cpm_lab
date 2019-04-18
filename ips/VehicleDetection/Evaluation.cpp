#include "Evaluation.h"


Evaluation::Evaluation(const std::vector<Situation> &situations, const std::vector<double> &durations)
{
	GlobalDataHelper::getInstance().getRealSituations(realSituations);

	//assert(realSituations.size() >= situations.size());

	calcSituations = situations;
	this->durations = durations;
}


Evaluation::~Evaluation()
{
}


void Evaluation::createFile(const std::string &filePath) {
	out = std::ofstream(filePath);
	writeToFile = true;
}

void Evaluation::printFile() {
	out.close();
}

void Evaluation::setMaxErrors(double maxErrorPos, double maxErrorOrientation) {
	this->maxErrorPos = maxErrorPos;
	maxErrorOri = maxErrorOrientation;
}

void Evaluation::calculateMinMaxMean(const std::vector<double> &values) {
	double min, max, mean, stdDev;
	ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(values, min, max, mean, stdDev);

	if (writeToFile) {
		out << "   Min    = " << min << std::endl;
		out << "   Max    = " << max << std::endl;
		out << "   Mean   = " << mean << std::endl;
		out << "   Stddev = " << stdDev << std::endl;
	} 
	else {
		std::cout << "   Min = " << min << std::endl;
		std::cout << "   Max = " << max << std::endl;
		std::cout << "   Mean= " << mean << std::endl;
		std::cout << "   Stddev = " << stdDev << std::endl;
	}
}

bool Evaluation::getRealLocationAtTime(const Time_Stamp &timestamp, const int vehicle, Location &location) {
	for (size_t i = startPoint; i < realSituations.size(); i++) {
		if (realSituations[i].timestamp == timestamp) {
			location = realSituations[i].mapping[vehicle];
			startPoint = i;
			return true;
		}
	}
	return false;
}


void Evaluation::calculatePositionError() {

	if (realSituations.empty()) {
		//no evaluation possible
		return;
	}

	int numOri = 0;
	int numPos = 0;

	for (size_t i = 0; i < calcSituations.size(); i++) {
		for (auto vehicle : calcSituations[i].mapping) {
			Location real_Loc;
			bool exists = getRealLocationAtTime(calcSituations[i].timestamp, vehicle.first, real_Loc);
			if (!exists) {
				continue;
			}

			double error_p = sqrt(ImageProcessingHelper::getInstance().calculateDistance(real_Loc.position, vehicle.second.position));
			double error_o = ImageProcessingHelper::getInstance().getDifference(real_Loc.orientation, vehicle.second.orientation);
			error_pos.push_back(error_p);
			error_orientation.push_back(error_o);

			if (error_p > maxErrorPos) {
				numPos++;
			}

			if (error_o > maxErrorOri) {
				std::cout << " id = " << vehicle.first << ": " << vehicle.second.position << " && " << vehicle.second.orientation << std::endl;
				numOri++;
			}
		}
	}

	if (!error_pos.empty()) {
		if (writeToFile) {
			out << "Position: " << std::endl;
		}
		else {
			std::cout << "Position: " << std::endl;
		}
		calculateMinMaxMean(error_pos);
	}
	
	if (!error_orientation.empty()) {
		if (writeToFile) {
			out << "Orientation: " << std::endl;
		}
		else {
			std::cout << "Orientation: " << std::endl;
		}
		calculateMinMaxMean(error_orientation);
	}
	
	double num_it = calcSituations.size()* calcSituations.front().mapping.size();

	if (numPos > 0) {
		if (writeToFile) {
			out << std::endl << "Position Error > " << maxErrorPos << " :" << std::endl;
			out << "   Absolute: " << numPos << std::endl;
			out << "   Relative: " << numPos / num_it << std::endl;
		}
		else {
			std::cout << std::endl << "Position Error > " << maxErrorPos << " :" << std::endl;
			std::cout << "   Absolute: " << numPos << std::endl;
			std::cout << "   Relative: " << numPos / num_it << std::endl;
		}
	}

	if (numOri > 0) {
		if (writeToFile) {
			out << std::endl << "Orientation Error > " << maxErrorOri << " :" << std::endl;
			out << "   Absolute: " << numOri << std::endl;
			out << "   Relative: " << numOri / num_it << std::endl;
		}
		else {
			std::cout << std::endl << "Orientation Error > " << maxErrorOri << " :" << std::endl;
			std::cout << "   Absolute: " << numOri << std::endl;
			std::cout << "   Relative: " << numOri / num_it << std::endl;
		}
	}
}

void Evaluation::calculateDuration( long long duration_sum) {

	if (durations.empty()) {
		return;
	}

	if (writeToFile) {
		out << "Overall Duration: " << duration_sum << " micro seconds" << std::endl;
		out << "Duration: " << std::endl;
	}
	else {
		std::cout << "Overall Duration: " << duration_sum << " micro seconds" << std::endl;
		std::cout << "Duration: " << std::endl;
	}
	calculateMinMaxMean(durations);
}

void Evaluation::calculateTimePointDistances(std::vector<double> &timePointDistances) {
	if (timePointDistances.empty()) {
		return;
	}

	if (writeToFile) {
		out << "Time Point Distances: " << std::endl;
	}
	else {
		std::cout << "Time Point Distances: " << std::endl;
	}
	calculateMinMaxMean(timePointDistances);
}

std::vector<double> Evaluation::getErrorPos() {
	return error_pos;
}

std::vector<double> Evaluation::getErrorOrientation() {
	return error_orientation;
}

void Evaluation::printEachSituation() {

	for (size_t i = 0; i < calcSituations.size(); i++) {
		std::cout << "Duration Situation: " << durations[i] << " micro seconds" << std::endl;

		auto situation = calcSituations[i];

		std::cout << "Situation at " << situation.timestamp << ":" << std::endl;
		for (auto it = situation.mapping.begin(); it != situation.mapping.end(); it++) {

			std::cout << "  id = " << it->first << std::endl;
			std::cout << "  position = " << it->second.position << std::endl;
			std::cout << "  orientation = " << it->second.orientation << std::endl << std::endl;
		}
		std::cout << std::endl;
	}
}

void Evaluation::illustrateRealSituations(size_t number, int id) {
	if (number > realSituations.size()) {
		number = realSituations.size();
	}

	illustrate(std::vector<Situation>(realSituations.begin(), next(realSituations.begin(), number)), id, false);
}

void Evaluation::illustrateCalcSituations(int id) {
	illustrate(calcSituations, id, false);
}


void Evaluation::illustrate(const std::vector<Situation> &situations, const int id, const bool saveToFile) {
	cv::Mat result = cv::Mat::zeros(cv::Size(2048, 2048), CV_8UC3);

	for (size_t i = 0; i < situations.size(); i++) {
		auto pair = situations[i].mapping.find(id);
		if (pair == situations[i].mapping.end()) {
			continue;
		}
		drawLocation(result, pair->second, cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0));
	}

	cv::resize(result, result, cv::Size(600, 600));

	std::string windowName = "ValidateResult_" + std::to_string(id);

	cv::namedWindow(windowName);
	cv::imshow(windowName, result);
	cv::waitKey(0);
	cv::destroyWindow(windowName);

	if (saveToFile) {
		std::vector<int> compression_params;
		compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		cv::imwrite("Path.png", result, compression_params);
	}

	result.release();
}

void Evaluation::drawLocation(const cv::Mat &image, const Location &location) {
	drawLocation(image, location, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255));
}

void Evaluation::drawLocation(const cv::Mat &image, const Location &location, const cv::Scalar &color1, const cv::Scalar &color2) {
	ImagePoint point = GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(location.position);
	cv::circle(image, point, 4, color1, cv::FILLED);

	double slope = tan(location.orientation);
	double b = location.position.y - (slope * location.position.x);

	double x;
	if (std::abs(location.orientation) > (M_PI / 2)) {
		x = location.position.x - 0.05;
	}
	else {
		x = location.position.x + 0.05;
	}

	double y = slope * x + b;
	WorldPoint oneStepForward(x, y, location.position.z);


	ImagePoint pointFoward = GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(oneStepForward);
	cv::arrowedLine(image, point, pointFoward, color2, 2, cv::LINE_4);
}
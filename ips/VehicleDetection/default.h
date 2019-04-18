#pragma once

#include "Constants.h"

#include <opencv2/opencv.hpp>
//#include <opencv2/aruco.hpp>

#include <memory>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <thread>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

#define MAX_IMAGES_PER_TARGET 100


typedef uint64_t Time_Stamp;

/**
	detection with LEDs or tags
*/
enum Mode {

	LED,
	TAG

};

/**
	2D Image point inherits from cv::Point2i
	overrides < for usage in maps etc.
*/
struct ImagePoint : cv::Point2i {

	ImagePoint() : cv::Point2i() {
	};

	ImagePoint(int x, int y) : cv::Point2i(x,y) {
	};

	ImagePoint(cv::Point2i point) : cv::Point2i(point) {
	}

	friend bool operator<(ImagePoint const& a, ImagePoint const& b)
	{
		return (a.x < b.x) || (a.x == b.x && a.y < b.y);
	}
};

/**
	3D World point inherits from cv::Point3d
	overrides < for usage in maps etc.
*/
struct WorldPoint : cv::Point3d {

	WorldPoint() : cv::Point3d() {
	};

	WorldPoint(double x, double y, double z) : cv::Point3d(x, y, z) {
	};

	WorldPoint(cv::Vec3d vec) : cv::Point3d(vec) {
	};

	friend bool operator<(WorldPoint const& a, WorldPoint const& b)
	{
		return (a.x < b.x) || (a.x == b.x && a.y < b.y) || (a.x == b.x && a.y == b.y && a.z < b.z);
	}

	friend WorldPoint operator+(WorldPoint const& a, WorldPoint const& b)
	{
		return WorldPoint(a.x + b.x, a.y + b.y, a.z + b.z);
	}
};

/**
	Location of the object described
	by position in 3D World space and
	orientation angle between right side of the vehicle
	and longer calibration line
*/
struct Location {
	WorldPoint position;
	double orientation;

	friend bool operator<(Location const& a, Location const& b)
	{
		return (a.position < b.position) || (a.position == b.position && a.orientation < b.orientation);
	}
};


/**
	describing an interval of numbers
	described by min and max value
	used for frequencies
*/
struct Interval {
	double min;
	double max;

	Interval() {
		min = 0;
		max = DBL_MAX;
	}

	Interval(const double min, const double max) {
		this->min = min;
		this->max = max;
	};

	bool in(const double x) {
		return min <= x && x <= max;
	};

	friend bool operator<(Interval const& a, Interval const& b)
	{
		return (a.min < b.min) || ((a.min == b.min) && (a.max < b.max));
	}

	friend bool operator==(Interval const& a, Interval const& b)
	{
		return (a.min == b.min) && (a.max == b.max);
	}

};

struct TimeInterval {
	Time_Stamp min;
	Time_Stamp max;

	TimeInterval(const Time_Stamp min, const Time_Stamp max) {
		this->min = min;
		this->max = max;
	};

	bool in(const Time_Stamp x) {
		return min <= x && x <= max;
	};

	bool isEmpty() {
		return max < min;
	};

	friend bool operator<(TimeInterval const& a, TimeInterval const& b)
	{
		return (a.min < b.min) || ((a.min == b.min) && (a.max < b.max));
	}

	friend bool operator==(TimeInterval const& a, TimeInterval const& b)
	{
		return (a.min == b.min) && (a.max == b.max);
	}

};

/**
	Characterisitic describing the identification
	here described by the time the led is on and the time
	it is off
*/
struct IdentificationCharacteristic {
	Interval onFrequency;
	Interval offFrequency;

	IdentificationCharacteristic(const Interval onFrequency, const Interval offFrequency) {
		this->onFrequency = onFrequency;
		this->offFrequency = offFrequency;
	};

	friend bool operator<(IdentificationCharacteristic const& a, IdentificationCharacteristic const& b)
	{
		return (a.onFrequency < b.onFrequency) || ((a.onFrequency == b.onFrequency) && (a.offFrequency < b.offFrequency)) ;
	}

};

struct Frequency {
	int on_time_ms;
	int off_time_ms;

	Frequency(const int on_time_ms, const int off_time_ms) {
		this->on_time_ms = on_time_ms;
		this->off_time_ms = off_time_ms;
	};

	friend bool operator<(Frequency const& a, Frequency const& b)
	{
		return (a.on_time_ms < b.on_time_ms) || ((a.on_time_ms == b.on_time_ms) && (a.off_time_ms < b.off_time_ms));
	}
};

/**
	Geometrie of the target described the WorldPoints of the
	position LEDs and the Points of the Identification LED
*/
struct Geometrie {
	std::vector<WorldPoint> positionPoints;
	std::vector<WorldPoint> identificationPoints;
	WorldPoint offsetMidPoint;

	size_t getNumberOfPositionPoints() {
		return positionPoints.size();
	};

	size_t getNumberOfIdentificationPoints() {
		return identificationPoints.size();
	};

	friend bool operator<(Geometrie &a, Geometrie &b) {
		return a.getNumberOfPositionPoints() < b.getNumberOfPositionPoints() 
			|| (a.getNumberOfPositionPoints() == b.getNumberOfPositionPoints()  
			&& a.getNumberOfIdentificationPoints() < b.getNumberOfIdentificationPoints());
	}
};


/**
	needed for detected object
*/
struct MatchedPoints {
	std::vector<ImagePoint> points;
	ImagePoint center;
	Time_Stamp timestamp;


	friend bool operator<(MatchedPoints const& a, MatchedPoints const& b)
	{
		return a.center < b.center;
	}


	friend bool operator==(MatchedPoints const& a, MatchedPoints const& b)
	{
		return a.center == b.center;
	}

};


typedef struct
{
	std::string data;
	std::vector<ImagePoint> location;
} Tag;


struct TimedImage {
	cv::Mat image;
	Time_Stamp timestamp;
	int id;

	TimedImage(const int id, const cv::Mat image) {
		this->id = id;
		this->image = image;
	};

	TimedImage(const int id, const cv::Mat image, const Time_Stamp &timestamp) {
		this->id = id;
		this->image = image;
		this->timestamp = timestamp;
	};
};

struct TimedPoints {
	std::vector<ImagePoint> points;
	Time_Stamp timestamp;

	TimedPoints(const std::vector<ImagePoint> &points) {
		this->points = points;
	};

	TimedPoints(const std::vector<ImagePoint> &points, const Time_Stamp &timestamp) {
		this->points = points;
		this->timestamp = timestamp;
	};

	TimedPoints(const TimedPoints &timepoints) {
		this->points = std::vector<ImagePoint>(timepoints.points.begin(), timepoints.points.end());
		this->timestamp = timepoints.timestamp;
	};
};


struct Situation {
	Time_Stamp timestamp;
	std::map<int, Location> mapping;
};
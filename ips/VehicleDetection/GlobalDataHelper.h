#pragma once

#include "default.h"
#include "CameraParameter.h"
#include "ImageProcessingHelper.h"


class GlobalDataHelper
{
public:

	//singelton
	static GlobalDataHelper& getInstance()
	{
		static GlobalDataHelper instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	~GlobalDataHelper() {}


	/**
		set start time of the system: time of system clock and ticks of camera
	*/
	void setStartTime(const uint64_t &startTime, const uint64_t startTick);

	/**
		get start time of camera where tickCount = 0
	*/
	uint64_t getStartTime();


	/**
		set the camera parameters
	*/
	void setCameraParameters(const std::shared_ptr<CameraParameter> cameraParameter);

	/**
		get the camera parameters
	*/
	std::shared_ptr<CameraParameter> getCameraParameters();


	/**
		set the geometrie of the vehicle
	*/
	void setGeometrie(const Geometrie &geometrie);

	/**
		get the geometrie of the vehicle
	*/
	Geometrie getGeometrie();


	/**
		set slope of reference line by giving two points on the line
	*/
	void setSlopeOfReferenceLine(const WorldPoint &start, const WorldPoint &end);

	/**
		get slope of reference line
	*/
	double getSlopeOfReferenceLine();


	/**
		set global constants of the system
	*/
	void setGlobalConst(const double height, const double roof_height, const int exposure_time);

	/**
		get global constants of the system
	*/
	GlobalConstants getGlobalConst();


	/**
		set constants for the LED mode
	*/
	void setLEDConst(const double cameraFrequency);

	/**		
		set tolerance to determine a point to be in the shortest distance (a back point)
		values given in pixels 
		test:  shortest_dis - tolerance_shortest_low < dis < shortest_dis + tolerance_shortest_high
	*/
	void setToleranceShortest(const double tolerance_shortest_low, const double tolerance_shortest_high);

	/**
		set tolerance to determine a point to be in the longest distance (point of the vehicle)
		relative value
		(1 + tolerance_longest) * longest_dis
	*/
	void setToleranceLongest(const double tolerance_longest);

	/**
		set tolerance to determine whether a point is in the range of an rectangle
		compare value for cv::pointPolygonTest if value is negative (outside the poylgon)
	*/
	void setToleranceRange(const double tolerance_range);

	/**
		set tolerance to determine whether the points build a valid vehicle geometry
	*/
	void setToleranceGeometry(const double tolerance_geometry);

	/**
		get constants for the LED mode
	*/
	LEDConstants getLEDConst();


	/**
		add a possible interval for the LED mode
	*/
	void addPossibleIntervalLED(const Interval &interval);

	/**
		get the possible intervals for the LED mode
	*/
	std::vector<Interval> getPossibleIntervalsLED();

	/**
		set real situations for evaluation with simulation
	*/
	void setRealSituations(const std::vector<Situation> &realSituations);

	/**
		get real situations for evaluation with simulation
	*/
	void getRealSituations(std::vector<Situation> &realSituations);

	/**
		set the identification map used
	*/
	void setIdentificationMap(const std::map<IdentificationCharacteristic, int> &vehicles);
	
	/**
		get identification map
	*/
	void getIdentificationMap(std::map<IdentificationCharacteristic, int> &vehicles);


	/**
		print min max mean and std dev of the different steps
	*/
	void evaluateDurations(std::ofstream &outputFile);

	/**
		add duration needed by the ImageHolder
	*/
	void addDurationImageHolder(long long duration);

	/**
		add duration needed by DetectedObjects
	*/
	void addDurationDetectedObjects(long long duration);

	/**
		add duration needed by SortedObjects
	*/
	void addDurationSortedObjects(long long duration);

	/**
		add duration needed by one target (time included in sorted objects
	*/
	void addDurationTarget(long long duration);

	/**
		add duration needed by all targets to provide their position and id
	*/
	void addDurationTargets(long long duration);

	/**
		set cancel variable to true
	*/
	void cancelProgram();

	/**
		check whether program was canceled
	*/
	bool isCanceled();

private:
	/**
		constrcutor only private
	*/
	GlobalDataHelper() {}

	/**
		guarantees that no further instance can
		be constructed via copy-constructor
	*/
	GlobalDataHelper(const GlobalDataHelper&);

	/**
		avoids für instances with copy
	*/
	GlobalDataHelper & operator = (const GlobalDataHelper &);


	/**
		set to true if the program should be stopped
		e.g. if no camera is connected
		-> never set to false during runtime!!
	*/
	bool cancel = false;

	/**
		start time of the camera where tickCount = 0
	*/
	time_t startTime;

	/**
		parameters of the camera
	*/
	std::shared_ptr<CameraParameter> cameraParameter;

	/**
		Geometrie of the vehicle
	*/
	Geometrie geometrie;

	/**
		slope of reference line
	*/
	double slopeOfReferenceLine;

	/**
		global constants of the system
	*/
	GlobalConstants globalConst;

	/**
		global constants for the LED Mode
	*/
	LEDConstants ledConst;

	/**
		possible intervals for LED mode
	*/
	std::vector<Interval> intervalsLED;

	/**
		real situations for the evaluation with the simulation
	*/
	std::vector<Situation> realSituations;

	/**
		identification map for the IDs and the characteristics
	*/
	std::map<IdentificationCharacteristic, int> identificationMap;

	/**
		durations needed by the ImageHodler
	*/
	std::vector<double> durationsImageHolder;

	/**
		durations needed by detected objects
	*/
	std::vector<double> durationsDetectedObjects;

	/**
		durations needed by sorted objects
	*/
	std::vector<double> durationsSortedObjects;

	/**
		durations needed by each target -> part of sorted objects
	*/
	std::vector<double> durationsTargetsEach;

	/**
		durations needed by the targets
	*/
	std::vector<double> durationsTargets;

	/**
		factor to convert from nanoseconds to the whished unit
	*/
	const double unit = 1000.0;
};


#pragma once

#include "Target.h"
#include "GlobalDataHelper.h"

class TargetLED :
	public Target
{
public:
	TargetLED(const int initial_negative_ID);
	~TargetLED();

	/**
		add points for the next image belonging to this target
		if points are plausible
	*/
	bool addNewImage(const TimedPoints &points);


	/**
		get the last images points corresponding to the target
		for matching the next image points
	*/
	TimedPoints getLastImage() {
		return *targetPointsFromLastImage;
	}	

private:

	//=============================== Variables ===================================================

	const size_t num_of_interpolation_points = 4;

	int plausibilityStartCounter = 0;

	/**
		ID of this target while actual ID is not determined
	*/
	int negative_ID;

	/**
		finding inital of the counters - removing first value
	*/
	bool initial_on = true;
	bool initial_off = true;
	
	/**
		points from the last image belonging to it
	*/
	std::shared_ptr<TimedPoints> targetPointsFromLastImage;

	/**
		counter for identification
	*/
	std::vector<size_t> onCounters;
	std::vector<size_t> offCounters;
	size_t counter = 0;
	bool ledOn;
	bool removeOnChange = false;


	/**
		saves the last locations for prediction
	*/
	std::vector<WorldPoint> locationHistory;
	std::vector<Time_Stamp> historyTimePoints;

	/**
		World Point of the target for the front right
	*/
	WorldPoint sideEnd;

	/**
		World Point of the target for the back right
	*/
	WorldPoint sideStart;

	/**
		World Point of the target for the back left
	*/
	WorldPoint thirdPoint;

	/**
		copies for calculation
	*/
	std::vector<size_t> onState;
	std::vector<size_t> offState;
	bool initial_on_copy;
	bool initial_off_copy;
	size_t counter_copy;
	bool ledOn_copy;
	bool removeOnChange_copy;

	//========================================== Functions ====================================================

	/**
		get vector of points only for positioning (not for identification)
	*/
	void getPointsForPositioning(const TimedPoints &points, std::vector<ImagePoint> &pointsForPositioning);

	/**
		based on image points calculate position and orientation of these points
	*/
	void calculatePositionAndOrientation(const TimedPoints &points, Location &location);

	WorldPoint calculatePositionWithOffset(const WorldPoint &position, double orientation);

	/**
		calculate current orientation in case of history at given timestamp and with calculated position
		otherwise with the found points
	*/
	double calculateOrientation(const Time_Stamp &timestamp, const Location &location);

	/**
		calculate orientation as average of the provided
	*/
	double calculateOrientationAsAverage(const double ori_side, const double ori_back, const double ori_diag);

	/**
		calculate orientation as median of the provided
	*/
	double calculateOrientationAsMedian(const double ori_side, const double ori_back, const double ori_diag);


	/**
		based on image points calculate identity of this target if they were added
	*/
	int calculateIdentification(const TimedPoints &points);

	/**
		increase counters for identification
	*/
	void increaseCounters(const std::vector<ImagePoint> &points);

	/**
		let the counters not exceed MAX_IMAGES_OF_TARGET values
	*/
	void limitVectors();

	/**
		adapt counters if calculations are plausible
		-> set to calculated results
	*/
	void adaptCounters(const TimedPoints &points);

	/**
		get on/off time from the counters
	*/
	double getTime(const std::vector<size_t> &counters);

	/**
		get identification characterisitc corresponding to the calculated frequencies
	*/
	IdentificationCharacteristic getCharacteristic(const double onFreqReal, const double offFreqReal);


	/**
		Plausibility Check of calculated ID and position
	*/
	bool isPlausible(const int id, const Location &location, const Time_Stamp &calculated_timestamp);

	/**
		check if calculated ID is plausible
	*/
	bool isIDPlausible(const int calculated_id);

	/**
		check if calculated position is plausible
	*/
	bool isPositionPlausible(const Location &calculated_location, const Time_Stamp &calculated_timestamp);

	/**
		check if calculated orientation is plausible
	*/
	bool isOrientationPlausible(const Location &calculated_location, const Time_Stamp &calculated_timestamp);

};


#pragma once

#include "TargetLED.h"

class SortedObjects
{
public:
	SortedObjects();
	~SortedObjects();

	/**
		add match points which should be sorted
	*/
	void addMatchPoints(const MatchedPoints &points);

	/**
		reorder the matchedPoints such that the points are mapped to 
		the correct target
	*/
	void matchToTargets(Targets &targets);

private:

	/**
		contain matchedPoints for each vehicle of one image
	*/
	std::vector<MatchedPoints> matchedPoints;


	/**
		condition to remove a target
	*/
	struct removeCond {

		Time_Stamp timeStamp;
		removeCond(const Time_Stamp &stamp) {
			timeStamp = stamp;
		}

		bool operator()(std::shared_ptr<Target> &target) {
			double updateTime = (timeStamp - target->getTimeStamp()) / 1000000.0;
			return updateTime > 300;
		}

	};

	/**
		next negative id to intialize Targets
	*/
	int negative_ID_counter = -1;


	/**
		find corresponding points that best fit the vehicle in the previousImage
	*/
	void searchBestFit(MatchedPoints &vehicle, std::map<ImagePoint, std::shared_ptr<TargetLED>> &matchedTargets, std::shared_ptr<TargetLED> &corresponding);

	/**
		get matched points of the last images stored in targets of current timestamp and outdated
	*/
	void getMatchedPointsTargets(std::map<ImagePoint, std::shared_ptr<TargetLED>> &mapping, std::map<ImagePoint, std::shared_ptr<TargetLED>> &outdatedMapping, const Targets &targets);

	/**
		initialze targets if they do not exist yet
	*/
	void initializeTargets(const std::vector<MatchedPoints> &matchedPoints, Targets &targets);

	/**
		remove target if not seen for 300ms
	*/
	void restrictTargets(Targets &targets);
};


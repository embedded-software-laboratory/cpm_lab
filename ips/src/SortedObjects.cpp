#include "SortedObjects.h"


SortedObjects::SortedObjects()
{
}


SortedObjects::~SortedObjects()
{
}

void SortedObjects::addMatchPoints(const MatchedPoints &points) {
	matchedPoints.emplace_back(points);
}

void SortedObjects::matchToTargets(Targets &targets) {

	struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

	assert(!matchedPoints.empty());
	
	if (matchedPoints.front().points.empty()) {
		//no points found in image
		targets.timestamp = matchedPoints.front().timestamp;
        
        clock_gettime(CLOCK_REALTIME, &t);
        uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
		GlobalDataHelper::getInstance().addDurationSortedObjects(t2 - t1);
		return;
	}

	restrictTargets(targets);

	if (targets.elements.empty()) {

		//first iteration
		initializeTargets(matchedPoints, targets);
    
        clock_gettime(CLOCK_REALTIME, &t);
        uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
		GlobalDataHelper::getInstance().addDurationSortedObjects(t2 - t1);
		return;
	}

	//only targets from the last time step
	std::map<ImagePoint, std::shared_ptr<TargetLED>> matchedTargets, outdatedMatchedTargets;
	getMatchedPointsTargets(matchedTargets, outdatedMatchedTargets, targets);

	for (MatchedPoints vehicle : matchedPoints) {
		std::shared_ptr<TargetLED> corresponding;
		searchBestFit(vehicle, matchedTargets, corresponding);
		if (corresponding == nullptr) {
			//number of targets increases or covered visible again
			searchBestFit(vehicle, outdatedMatchedTargets, corresponding);
			if (corresponding == nullptr) {
				std::shared_ptr<TargetLED> target = std::make_shared<TargetLED>(negative_ID_counter);
				target->addNewImage(TimedPoints(vehicle.points, vehicle.timestamp));
				targets.elements.push_back(target);
				negative_ID_counter--;
				continue;
			}
			corresponding->addNewImage(TimedPoints(vehicle.points, vehicle.timestamp));
		}
		else {
			corresponding->addNewImage(TimedPoints(vehicle.points, vehicle.timestamp));
		}
	}
	targets.timestamp = matchedPoints.front().timestamp;

    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
	GlobalDataHelper::getInstance().addDurationSortedObjects(t2-t1);
}


void SortedObjects::initializeTargets(const std::vector<MatchedPoints> &matchedPoints, Targets &targets) {
	for (MatchedPoints vehicle : matchedPoints) {
		std::shared_ptr<TargetLED> target = std::make_shared<TargetLED>(negative_ID_counter);
		target->addNewImage(TimedPoints(vehicle.points, vehicle.timestamp));
		targets.elements.push_back(target);
		negative_ID_counter--;
	}
	targets.timestamp = matchedPoints.front().timestamp;
}


void SortedObjects::getMatchedPointsTargets(std::map<ImagePoint, std::shared_ptr<TargetLED>> &mapping, std::map<ImagePoint, std::shared_ptr<TargetLED>> &outdatedMapping, const Targets &targets) {

	for (std::shared_ptr<Target> target : targets.elements) {

		if (std::shared_ptr<TargetLED> targetLED = std::dynamic_pointer_cast<TargetLED>(target)) {

			ImagePoint center = ImageProcessingHelper::getInstance().calculateCenter(targetLED->getLastImage().points);

			if (target->getTimeStamp() != targets.timestamp) {
				//not a current target -> outdated
				outdatedMapping[center] = targetLED;
			}
			else {
				mapping[center] = targetLED;
			}
		}
	}
}

void SortedObjects::restrictTargets(Targets &targets) {

	removeCond cond(targets.timestamp);
	targets.elements.remove_if(cond);
}


void SortedObjects::searchBestFit(MatchedPoints &vehicle, std::map<ImagePoint, std::shared_ptr<TargetLED>> &matchedTargets, std::shared_ptr<TargetLED> &corresponding) {

	double distance_min = DBL_MAX;

	std::pair<ImagePoint, std::shared_ptr<TargetLED>> corresPair;

	for (std::pair<ImagePoint, std::shared_ptr<TargetLED>> matchedTarget : matchedTargets) {

		double distance = ImageProcessingHelper::getInstance().calculateDistance(vehicle.center, matchedTarget.first);

		if (distance < distance_min) {
			distance_min = distance;
			corresPair = matchedTarget;
		}
	}

	if (distance_min == DBL_MAX) {
		return;
	}

	double max_possible_distance = ImageProcessingHelper::getInstance().getMaximalPossibleDistance(vehicle.center, corresPair.second->getTimeStamp(), matchedPoints.front().timestamp);

	if (distance_min < max_possible_distance) {

		corresponding = corresPair.second;
		matchedTargets.erase(corresPair.first);
	}
}

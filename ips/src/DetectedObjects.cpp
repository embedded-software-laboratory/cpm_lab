#include "DetectedObjects.h"

void DetectedObjects::addPointsForImage(const TimedPoints &points) {
	timedPoints = std::make_shared<TimedPoints>(points);
}


void DetectedObjects::sortPoints(std::shared_ptr<SortedObjects> &sortedObjects) {

    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);


	sortedObjects = std::make_shared<SortedObjects>();

	if (timedPoints->points.empty()) {
		MatchedPoints points;
		points.timestamp = timedPoints->timestamp;
		sortedObjects->addMatchPoints(points);


        clock_gettime(CLOCK_REALTIME, &t);
        uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
		GlobalDataHelper::getInstance().addDurationDetectedObjects(t2 - t1);
		return;
	}

	//cluster points
	std::vector<std::vector<ImagePoint>> clusters;
	clusterPoints(clusters);

	std::vector<std::shared_ptr<std::vector<MatchedPoints>>> vehicles;

	//start thread for each cluster
	std::vector<std::thread> threads;
	for (size_t i = 0; i < clusters.size(); i++) {
		//create vehicle vectors per cluster

		std::shared_ptr<std::vector<MatchedPoints>> vehiclesInCluster = std::make_shared<std::vector<MatchedPoints>>();

        threads.push_back(std::thread([this, clusters, vehiclesInCluster, i] { this->sorting(clusters[i], *vehiclesInCluster); }));

		vehicles.push_back(vehiclesInCluster);
	}

	//join all threads
	for (size_t i = 0; i < threads.size(); i++) {
		threads[i].join();
	}

	//merge results of each thread

	std::vector<ImagePoint> addedPoints;
	for (std::shared_ptr<std::vector<MatchedPoints>> vehiclePerThread : vehicles) {
		for (MatchedPoints vehicle : *vehiclePerThread) {
			if (find(addedPoints.begin(), addedPoints.end(), vehicle.points.front()) == addedPoints.end()) {
				sortedObjects->addMatchPoints(vehicle);
				addedPoints.insert(addedPoints.end(), vehicle.points.begin(), vehicle.points.end());
			}
		}
	}
	
	if (addedPoints.empty()) {
		MatchedPoints points;
		points.timestamp = timedPoints->timestamp;
		sortedObjects->addMatchPoints(points);
	}

    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
	GlobalDataHelper::getInstance().addDurationDetectedObjects(t2 - t1);
}

void DetectedObjects::clusterPoints(std::vector<std::vector<ImagePoint>> &clusters) {

	Geometrie geometrie = GlobalDataHelper::getInstance().getGeometrie();

	std::vector<ImagePoint> remaining = timedPoints->points;

	while (!remaining.empty()) {
		ImagePoint start = remaining.front();
		std::vector<ImagePoint> cluster;
		cluster.push_back(start);

		size_t i = 0;
		while (i < cluster.size()) {
			double distanceVehicle = ImageProcessingHelper::getInstance().getLongestDistance(geometrie, cluster[i].x, cluster[i].y);
			std::vector<ImagePoint> in_dis = ImageProcessingHelper::getInstance().getPointsInDistance(cluster[i], distanceVehicle, timedPoints->points);
			for (ImagePoint point : in_dis) {
				if (find(cluster.begin(), cluster.end(), point) == cluster.end()) {
					cluster.push_back(point);
				}
			}
			i++;
		}

		clusters.push_back(cluster);
		for (ImagePoint point : cluster) {
			auto point_it = find(remaining.begin(), remaining.end(), point);
			if (point_it != remaining.end()) {
				remaining.erase(point_it);
			}
		}
	}
}

void DetectedObjects::sorting(const std::vector<ImagePoint> &cluster, std::vector<MatchedPoints> &vehicles) {

	Geometrie geometrie = GlobalDataHelper::getInstance().getGeometrie();
	std::shared_ptr<VehicleMapping> vehicleMapping = std::make_shared<VehicleMapping>(cluster, geometrie);

	std::vector<ImagePoint> sortedAllPoints(cluster.begin(), cluster.end());
	std::sort(sortedAllPoints.begin(), sortedAllPoints.end());

	RemainingPoints remaining(sortedAllPoints);
	std::vector<ImagePoint> remainingAllPoints = sortedAllPoints;

	size_t previous_size = 0;
	int same_size_in_row = 0;
	while ((same_size_in_row < 5) && (remaining.getRemaining().size() >= geometrie.getNumberOfPositionPoints()) && (!remainingAllPoints.empty())) {
		if (previous_size == remaining.getRemaining().size()) {
			same_size_in_row++;
		}
		else {
			same_size_in_row = 0;
		}
		previous_size = remaining.getRemaining().size();
		for (ImagePoint point : remainingAllPoints) {
			if (!remaining.isMarked(point)) {
				bool found_vehicle = sortNonConflictingPoints(vehicleMapping, remaining, point, geometrie, vehicles);
				if (found_vehicle) {
					vehicleMapping->refineMapping(remaining.getRemaining());
				}
			}
		}

		//std::vector<ImagePoint> intersection;
		//std::vector<ImagePoint> remainingPoints = remaining.getRemaining();
		//std::set_intersection(remainingPoints.begin(), remainingPoints.end(), sortedAllPoints.begin(), sortedAllPoints.end(), std::back_inserter(intersection));
		remainingAllPoints = remaining.getRemaining();
		std::sort(remainingAllPoints.begin(), remainingAllPoints.end(), VehicleMapping::cmp(vehicleMapping->getNumInVehicleDistanceMapping()));
		std::reverse(remainingAllPoints.begin(), remainingAllPoints.end());

	}
}

bool DetectedObjects::sortNonConflictingPoints(
								std::shared_ptr<VehicleMapping> &pointMapping,
								RemainingPoints &remaining,
								const ImagePoint &point,
								Geometrie &geometrie,
								std::vector<MatchedPoints> &foundVehicles) {

	size_t mappingSize = pointMapping->getNumberOfPointsInVehicleDistance(point);

	if (mappingSize < geometrie.getNumberOfPositionPoints()) {
		//filter out this point -> does not belong to any vehicle
		remaining.markPoint(point);
		return true;
	}

	if (mappingSize == geometrie.getNumberOfPositionPoints()) {
		std::vector<ImagePoint> intersec = pointMapping->getIntersection(point, remaining.getRemaining());

		if (intersec.size() == mappingSize) {
			if (validateDistances(point, geometrie, intersec)) {
				foundVehicle(intersec, remaining, foundVehicles);
			}
			else {
				remaining.markPoint(point);
			}
		}
		else {
			remaining.markPoint(point);
			assert(1 != 0);
		}
		return true;
	}

	if (mappingSize == geometrie.getNumberOfPositionPoints() + geometrie.getNumberOfIdentificationPoints()) {
		
		std::vector<ImagePoint> intersec = pointMapping->getIntersection(point, remaining.getRemaining());

		if (intersec.size() == mappingSize) {
			if (validateDistances(point, geometrie, intersec)) {
				foundVehicle(intersec, remaining, foundVehicles);
				return true;
			}
		}
	}

	return false;
}


void DetectedObjects::foundVehicle(
	const std::vector<ImagePoint> &intersec,
	RemainingPoints &remaining,
	std::vector<MatchedPoints> &foundVehicles) {

	MatchedPoints matchedPoint;
	matchedPoint.points = intersec;
	matchedPoint.center = ImageProcessingHelper::getInstance().calculateCenter(intersec);
	matchedPoint.timestamp = timedPoints->timestamp;
	foundVehicles.push_back(matchedPoint);

	for (ImagePoint intersecPoint : intersec) {
		remaining.markPoint(intersecPoint);
	}
}

bool DetectedObjects::validateDistances(
	const ImagePoint &point,
	Geometrie &geometrie,
	std::vector<ImagePoint> &intersec) {

	std::vector<double> distanceInVehicle = ImageProcessingHelper::getInstance().getSortedDistancesInGeometry(point, geometrie);

	std::vector<double> distancesInIntersection;
	for (std::vector<ImagePoint>::iterator it1 = intersec.begin(); it1 != intersec.end(); it1++) {
		for (std::vector<ImagePoint>::iterator it2 = next(it1, 1); it2 != intersec.end(); it2++) {
			double dis = ImageProcessingHelper::getInstance().calculateDistance(*it1, *it2);
			distancesInIntersection.push_back(dis);
		}
	}
	std::sort(distancesInIntersection.begin(), distancesInIntersection.end());

	if (intersec.size() == geometrie.getNumberOfPositionPoints()) {
		//only compare values of position points: thus smallest and two biggest distances
		std::vector<size_t> indicies = { 0, 4, 5 };
		size_t j = 0;
		for (size_t i : indicies) {
			if (std::abs(distancesInIntersection[j] - distanceInVehicle[i]) > GlobalDataHelper::getInstance().getLEDConst().TOLERANCE_GEOMETRY_DISTANCES ) {
				return false;
			}
			j++;
		}
	}
	else {
		//compare all values
		assert(distanceInVehicle.size() == distancesInIntersection.size());

		for (size_t i = 0; i < distanceInVehicle.size(); i++) {
			if (std::abs(distancesInIntersection[i] - distanceInVehicle[i]) > GlobalDataHelper::getInstance().getLEDConst().TOLERANCE_GEOMETRY_DISTANCES) {
				return false;
			}
		}
	}
	return true;
}

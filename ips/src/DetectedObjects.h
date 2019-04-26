#pragma once

#include "SortedObjects.h"
#include "VehicleMapping.h"
#include "RemainingPoints.h"

class DetectedObjects
{
public:
	DetectedObjects() {};
	~DetectedObjects() {};

	/**
		add detected points for the next image
	*/
	void addPointsForImage(const TimedPoints &points);

	/**
		all points consider to belong to one target object are saved in one vector
		the list of all targets in one image is saved in the vector of those vectors
	*/
	void sortPoints(std::shared_ptr<SortedObjects> &sortedObjects);

	/**
		set id of the object to track the order of objects
	*/
	void setID(int id) {
		this->id = id;
	}

	/**
		get id of the object
	*/
	int getID() {
		return id;
	}


//================== function for threads =======================================================

	/**
		sort points in one cluster
	*/
	void sorting(const std::vector<ImagePoint> &cluster, std::vector<MatchedPoints> &vehicles);


private:


	//========================= Variables =============================================================

	/**
		id of the object to track the order
	*/
	int id;

	/**
		contains list of vector of points found in one images
		thus list contains all points found in all images
	*/
	std::shared_ptr<TimedPoints> timedPoints;



	//========================= Functions ==========================================================================

	/**
		create clusters from the provided timedpoints
	*/
	void clusterPoints(std::vector<std::vector<ImagePoint>> &clusters);

	/**
		sort points where one point only see points corresponding to its vehicle
	*/
	bool sortNonConflictingPoints(
		std::shared_ptr<VehicleMapping> &pointMapping,
		RemainingPoints &remaining,
		const ImagePoint &point,
		Geometrie &geometrie,
		std::vector<MatchedPoints> &foundVehicles);

	/**
		check if the distances in the intersection correlate to the distances in the geometry
		If not the points do not belong to one vehicle
	*/
	bool validateDistances(const ImagePoint &point, Geometrie &geometrie, std::vector<ImagePoint> &intersec);

	/**
		a vehicle was found: save this point from the intersection and mark them in the remaining points
	*/
	void foundVehicle(const std::vector<ImagePoint> &intersec, RemainingPoints &remaining, std::vector<MatchedPoints> &foundVehicles);

};

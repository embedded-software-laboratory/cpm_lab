#pragma once
#include "default.h"
#include "ImageProcessingHelper.h"


class VehicleMapping
{

private:

	//============================= helper structs ================================

	/**
		option of points to be a vehicle back or an end
	*/
	struct Option {
		//pairs that can be vehicle backs
		std::vector<std::pair<ImagePoint, ImagePoint>> vehicle_backs;

		//ends that belong to the vehicle backs
		std::vector<ImagePoint> ends;

		//ranges that the vehicle backs see
		std::vector<std::vector<cv::Point2f>> vehicleRanges;
	};

	/**
		back wheels of a vehicle
	*/
	struct VehicleBack {
		//pair of back points
		std::pair<ImagePoint, ImagePoint> position;

		//range which can be seen from the vehicle backs
		std::vector<cv::Point2f> range;

		friend bool operator<(VehicleBack const& a, VehicleBack const& b)
		{
			return (a.position.first < b.position.first) 
				|| (a.position.first == b.position.first && a.position.second < b.position.second);
		}

		friend bool operator==(VehicleBack const& a, VehicleBack const& b)
		{
			return (a.position.first == b.position.first) && (a.position.second == b.position.second);
		}
	};

	/**
		number of points in the vehicle distance of a point
	*/
	std::map<ImagePoint, size_t> numInVehicleDistance;

	/**
		number of points in the back distance of a point
	*/
	std::map<ImagePoint, size_t> numInBackDistance;

	/**
		points in the back distance of a point
	*/
	std::map<ImagePoint, std::vector<ImagePoint>> pointsInBackDistance;

	/**
		if a point belongs to a vehicle back it is saved in this mapping
		with its corresponding back
	*/
	std::map<ImagePoint, VehicleBack> backPointsMapping;

	/**
		points in the vehicle range of a vehicle back
	*/
	std::map<VehicleBack, std::vector<ImagePoint>> pointsInVehicleRangeOfBacks;

	/**
		points in the vehicle range of a point which is not a back point
	*/
	std::map<ImagePoint, std::vector<ImagePoint>> pointsInVehicleRangeOfNonBack;


	//============================== helper functions ===============================

	/**
		determine all points from the given vector of points that lay in the specified rectangle
	*/
	std::vector<ImagePoint> getPointsInRectangle(const std::vector<ImagePoint> &allPoints, const std::vector<cv::Point2f> &rectangle);

	/**
		compute the vehicle backs from the pointsInBackDistance Mapping and save them to the backPointMapping
	*/
	void getVehicleBacksFromMapping(const std::vector<ImagePoint> &remainingAllPoints, Geometrie &geometrie);

	/**
		if x sees y then y has to see x too (in the pointsInBackDistance Mapping)
	*/
	void smoothBackMapping();

	/**
		get the vector of points in the vehicle distance for the given point.
		Depending on whether this point is a back point or not
	*/
	std::vector<ImagePoint> getVehicleMappingPoint(const ImagePoint &point);

	/**
		set the vector of points in the vehicle distance for the given point.
		Depending on whether this point is a back point or not to the pointsInVehicleRangeOfBacks
		or pointsInVehicleRangeOfNonBacks Mapping
	*/
	void setVehicleMappingPoint(const ImagePoint &point, const std::vector<ImagePoint> &mapping);

	/**
		compute for two backPoints point1 and point2 the rectangle in which the corresponding points
		have to be
	*/
	void calculateVehicleRange(const ImagePoint &point1, const ImagePoint &point2, const Geometrie &geometrie, std::vector<cv::Point2f> &contour);

	/**
		compute the vehicle back from the pointsInBackDistance for the specified point and save it to
		the backPointsMapping
	*/
	void calculateVehicleBack(std::map<ImagePoint, bool> &marked, const ImagePoint &point, const std::vector<ImagePoint> &allPoints, Geometrie &geometrie);

	/**
		compute the option to be a vehicle back for the specified point and save it to the backPointsMapping
		needed if a point has two other points in its pointsInBackDistance Mapping
	*/
	void calculateOption(const ImagePoint &point, Option &option, std::map<ImagePoint, bool> &marked, const Geometrie &geometrie, const std::vector<ImagePoint> &inRange);

	/**
		compute the actual vehicle backs from the options of being a vehicle back
	*/
	void calculateVehicleBacksFromOptions(const std::vector<Option> &options, const std::vector<ImagePoint> &allPoints);

	/**
		check whether a point is in the range of a rectangle
	*/
	bool isInRange(const std::vector<cv::Point2f> &contour, const ImagePoint &point);


public:
	
	VehicleMapping(const std::vector<ImagePoint> &allPoints, Geometrie &geometrie);
	~VehicleMapping() {};

	/**
		compute the intersection of the vehicle mapping of the provided point with the mappings of these points
		this intersection is intersected with the remaining points
	*/
	std::vector<ImagePoint> getIntersection(const ImagePoint &point, const std::vector<ImagePoint> &remaining);

	/**
		get the number of points being in the vehicle distance of the specified point
	*/
	size_t getNumberOfPointsInVehicleDistance(const ImagePoint &point);

	/**
		refine the vehicle Mappings to only contain points contained in the remaining points
	*/
	void refineMapping(const std::vector<ImagePoint> &remaining);

	/**
		provide the mapping describing the number of points in the vehicle distance of a point
	*/
	std::map<ImagePoint, size_t> getNumInVehicleDistanceMapping() {
		return numInVehicleDistance;
	}

	/**
		compare class to sort according to the size of the mapping
	*/
	struct cmp {

		std::map<ImagePoint, size_t> mapping;

		cmp(const std::map<ImagePoint, size_t> &mapping) {
			this->mapping = mapping;
		}

		bool operator()(const ImagePoint &a, const ImagePoint &b) {
			return mapping[a] > mapping[b];
		}

	};

};


#pragma once

#include "default.h"
#include "GlobalDataHelper.h"

class ImageProcessingHelper
{
public:

	//singelton
	static ImageProcessingHelper& getInstance()
	{
		static ImageProcessingHelper instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	~ImageProcessingHelper() {}

	/**
		calculate distances between two image points
	*/
	double calculateDistance(const ImagePoint &point1, const ImagePoint &point2);

	/**
		calculate distances between two world points
	*/
	double calculateDistance(const WorldPoint &point1, const WorldPoint &point2);

	/**
		calculate longest distance in geometrie in pixels for specified position in image
	*/
	double getLongestDistance(const Geometrie &geometrie, const int x, const int y);

	/**
		calculate the short distance in geometrie in pixels for the specified position
	*/
	double getShortDistance(const Geometrie &geometrie, const int x, const int y);

	/**
		calculate the sum of the distances of the specified point to all the other points
	*/
	double getSumDistance(const std::vector<ImagePoint> &points, const ImagePoint &point);

	/**
		calculate maximal pixel distance of a vehicle driving in specified time for the specified position
	*/
	double getMaximalPossibleDistance(const ImagePoint &start, const Time_Stamp &begin, const Time_Stamp &end);


	/**
		determine all points in up to the specified distance to the given point from the vector of points
	*/
	std::vector<ImagePoint> getPointsInDistance(const ImagePoint &point, const double distance, const std::vector<ImagePoint> &allPoints);

	/**
		determine all points in exactly the specified distance to the given point from the vector of points with the specified tolerances
	*/
	std::vector<ImagePoint> getPointsInDistance(const ImagePoint &point, const double distance, const std::vector<ImagePoint> &allPoints, double tolerance_lower, double tolerance_upper);

	/**
		determine the distances in the vehicle geometry between the points at the position of the specified point
	*/
	std::vector<double> getSortedDistancesInGeometry(const ImagePoint &point, const Geometrie &geometry);

	/**
		calculate center of point cloud
	*/
	ImagePoint calculateCenter(const std::vector<ImagePoint> &points);
	WorldPoint calculateCenter(const std::vector<WorldPoint> &region);



	/**
		calculate orientation from three points in World Coordinate to x-axis
	*/
	double calculateOrientation(const ImagePoint &sideStart, const ImagePoint &sideEnd, const ImagePoint &thirdPoint, const double height);


	/**
		calculate difference between angles: maximal 180 degree
	*/
	double getDifference(double angle1, double angle2);

	/**
		sort three LED points to decide which is the start and the end of the side and which is the third
	*/
	void sortPointsForPositioning(const std::vector<ImagePoint> &pointsForPositioning, WorldPoint &sideStart, WorldPoint &sideEnd, WorldPoint &thirdPoint);

	/**
		calculate min max mean and the standard deviation of a vector of double values
	*/
	void calculateMinMaxMeanStdDev(const std::vector<double> &values, double &min, double &max, double &mean, double &stdDev);

	/**
		calculate values needed to draw a boxplot
	*/
	void calculateBoxplot(const std::vector<double> &values, double &median, double &upperQuartil, double &lowerQuartil, double &upperWhisker, double &lowerWhisker);

private:
	/**
		constrcutor only private
	*/
	ImageProcessingHelper() {}

	/** 
		guarantees that no further instance can 
		be constructed via copy-constructor
	*/
	ImageProcessingHelper(const ImageProcessingHelper&);

	/**
		avoids für instances with copy
	*/
	ImageProcessingHelper & operator = (const ImageProcessingHelper &);


	/**
		helper function for sorting points
	*/
	void calculateStartAndThird(const WorldPoint &candidate1, const WorldPoint &candidate2, WorldPoint &sideEnd, WorldPoint &sideStart, WorldPoint &thirdPoint);

};


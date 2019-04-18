#pragma once
#include "default.h"

class RemainingPoints
{
public:
	RemainingPoints(const std::vector<ImagePoint> &allPoints);
	~RemainingPoints() {}

	void markPoint(const ImagePoint &point);

	std::vector<ImagePoint> getRemaining();

	bool isMarked(const ImagePoint &point);


private:

	bool updated = false;

	std::vector<ImagePoint> allPoints;

	std::vector<ImagePoint> markedPoints;

	std::vector<ImagePoint> remaining;

	std::map<ImagePoint, bool> marked;

};


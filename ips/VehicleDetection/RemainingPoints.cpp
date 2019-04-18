#include "RemainingPoints.h"


RemainingPoints::RemainingPoints(const std::vector<ImagePoint> &allPoints)
{
	this->allPoints = allPoints;
	this->remaining = allPoints;
}


void RemainingPoints::markPoint(const ImagePoint &point) {
	marked[point] = true;

	markedPoints.push_back(point);
	std::sort(markedPoints.begin(), markedPoints.end());

	updated = true;
}

std::vector<ImagePoint> RemainingPoints::getRemaining() {
	if (updated) {
		updated = false;

		std::vector<ImagePoint> markedCopy = markedPoints;

		remaining.clear();

        std::set_difference(allPoints.begin(), allPoints.end(), 
                            markedCopy.begin(), markedCopy.end(),
                            std::inserter(remaining, remaining.begin()));
	}

	return remaining;
}

bool RemainingPoints::isMarked(const ImagePoint &point) {
	return marked[point];
}
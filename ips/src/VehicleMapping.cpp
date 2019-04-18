#include "VehicleMapping.h"

VehicleMapping::VehicleMapping(const std::vector<ImagePoint> &allPoints, Geometrie &geometrie) {

	for (ImagePoint point : allPoints) {
		double distanceBack = ImageProcessingHelper::getInstance().getShortDistance(geometrie, point.x, point.y);
		pointsInBackDistance[point] = ImageProcessingHelper::getInstance().getPointsInDistance(point, distanceBack, allPoints, GlobalDataHelper::getInstance().getLEDConst().TOLERANCE_SHORTEST_LOW, GlobalDataHelper::getInstance().getLEDConst().TOLERANCE_SHORTEST_HIGH);
		numInBackDistance[point] = pointsInBackDistance[point].size();
	}

	smoothBackMapping();

	getVehicleBacksFromMapping(allPoints, geometrie);

	for (ImagePoint point : allPoints) {
		if (backPointsMapping.find(point) == backPointsMapping.end()) {
			//no back point - circle
			double distanceVehicle = ImageProcessingHelper::getInstance().getLongestDistance(geometrie, point.x, point.y);
			pointsInVehicleRangeOfNonBack[point] = ImageProcessingHelper::getInstance().getPointsInDistance(point, distanceVehicle, allPoints);
			numInVehicleDistance[point] = pointsInVehicleRangeOfNonBack[point].size();
		}
	}
}


std::vector<ImagePoint> VehicleMapping::getIntersection(const ImagePoint &point, const std::vector<ImagePoint> &remaining) {

	std::vector<ImagePoint> intersec(remaining.begin(), remaining.end());
	for (ImagePoint candidate : getVehicleMappingPoint(point)) {
		std::vector<ImagePoint> mapping = getVehicleMappingPoint(candidate);

		std::vector<ImagePoint> intersection;
		std::set_intersection(intersec.begin(), intersec.end(), mapping.begin(), mapping.end(), std::back_inserter(intersection));
		intersec = intersection;
	}

	return intersec;
}

size_t VehicleMapping::getNumberOfPointsInVehicleDistance(const ImagePoint &point) {
	return numInVehicleDistance[point];
}

void VehicleMapping::refineMapping(const std::vector<ImagePoint> &remaining) {

	std::map<ImagePoint, bool> marked;

	for (ImagePoint point : remaining) {
		if (marked[point]) {
			continue;
		}

		std::vector<ImagePoint> mapping = getVehicleMappingPoint(point);

		std::vector<ImagePoint> intersection;
		std::set_intersection(remaining.begin(), remaining.end(), mapping.begin(), mapping.end(), std::back_inserter(intersection));

		setVehicleMappingPoint(point, intersection);

		numInVehicleDistance[point] = intersection.size();

		if (backPointsMapping.find(point) == backPointsMapping.end()) {
			//no back point
			marked[point] = true;
		}
		else {
			marked[backPointsMapping[point].position.first] = true;
			marked[backPointsMapping[point].position.second] = true;
		}
		
	}
}





//===================== helper ===============================================================

std::vector<ImagePoint> VehicleMapping::getPointsInRectangle(const std::vector<ImagePoint> &allPoints, const std::vector<cv::Point2f> &rectangle) {
	
	std::vector<ImagePoint> pointsInRectangle;

	for (ImagePoint candidate : allPoints) {

		if (isInRange(rectangle, candidate)) {
			pointsInRectangle.emplace_back(candidate);
		}
	}

	std::sort(pointsInRectangle.begin(), pointsInRectangle.end());

	return pointsInRectangle;
}

std::vector<ImagePoint> VehicleMapping::getVehicleMappingPoint(const ImagePoint &point) {
	if (backPointsMapping.find(point) == backPointsMapping.end()) {
		//no back point
		return pointsInVehicleRangeOfNonBack[point];
	}
	return pointsInVehicleRangeOfBacks[backPointsMapping[point]];
}

void VehicleMapping::setVehicleMappingPoint(const ImagePoint &point, const std::vector<ImagePoint> &mapping) {
	if (backPointsMapping.find(point) == backPointsMapping.end()) {
		//no back point
		pointsInVehicleRangeOfNonBack[point] = mapping;
		return;
	}
	pointsInVehicleRangeOfBacks[backPointsMapping[point]] = mapping;
}

void VehicleMapping::getVehicleBacksFromMapping(
	const std::vector<ImagePoint> &remainingAllPoints,
	Geometrie &geometrie) {

	std::map<ImagePoint, bool> marked;

	std::vector<ImagePoint> resortedPoints(remainingAllPoints.begin(), remainingAllPoints.end());
	sort(resortedPoints.begin(), resortedPoints.end(), cmp(numInBackDistance));

	std::vector<Option> options;

	for (ImagePoint point : resortedPoints) {
		if (marked[point]) {
			continue;
		}

		assert(pointsInBackDistance[point].size() <= 2);

		if (pointsInBackDistance[point].size() == 2) {
			Option option;
			calculateOption(point, option, marked, geometrie, pointsInBackDistance[point]);
			options.push_back(option);
		}

		if (pointsInBackDistance[point].size() == 1) {
			calculateVehicleBack(marked, point, remainingAllPoints, geometrie);
		}
	}

	if (!options.empty()) {
		calculateVehicleBacksFromOptions(options, remainingAllPoints);
	}
}


void VehicleMapping::smoothBackMapping() {
	for (auto map : pointsInBackDistance) {
		for (ImagePoint point : map.second) {
			if (find(pointsInBackDistance[point].begin(), pointsInBackDistance[point].end(), map.first) == pointsInBackDistance[point].end()) {
				//add map.first to pointMapping[point]
				pointsInBackDistance[point].insert(pointsInBackDistance[point].end(), map.first);
				std::sort(pointsInBackDistance[point].begin(), pointsInBackDistance[point].end());
				numInBackDistance[point] = pointsInBackDistance[point].size();
			}
		}
	}
}

void VehicleMapping::calculateVehicleBacksFromOptions(
	const std::vector<Option> &options,
	const std::vector<ImagePoint> &allPoints) {

	std::vector<size_t> matchedOptions;
	bool matched = false;

	for (size_t i = 0; i < options.size(); i++) {
		if (find(matchedOptions.begin(), matchedOptions.end(), i) != matchedOptions.end()) {
			continue;
		}
		matched = false;
		for (size_t j = 0; j < options[i].vehicleRanges.size(); j++) {
			for (size_t k = i + 1; k < options.size(); k++) {
				if (find(matchedOptions.begin(), matchedOptions.end(), k) != matchedOptions.end()) {
					continue;
				}
				for (size_t l = 0; l < options[k].ends.size(); l++) {
					if (isInRange(options[i].vehicleRanges[j], options[k].ends[l])) {
						//Check if match in other direction
						if (isInRange(options[k].vehicleRanges[l], options[i].ends[j])) {
							//vehicle found

							VehicleBack back_1;
							back_1.position = options[i].vehicle_backs[j];
							back_1.range = options[i].vehicleRanges[j];

							std::vector<ImagePoint> pointsInRange1 = getPointsInRectangle(allPoints, back_1.range);
							pointsInVehicleRangeOfBacks[back_1] = pointsInRange1;
							numInVehicleDistance[options[i].vehicle_backs[j].first] = pointsInVehicleRangeOfBacks[back_1].size();
							numInVehicleDistance[options[i].vehicle_backs[j].second] = pointsInVehicleRangeOfBacks[back_1].size();

							backPointsMapping[options[i].vehicle_backs[j].first] = back_1;
							backPointsMapping[options[i].vehicle_backs[j].second] = back_1;


							VehicleBack back_2;
							back_2.position = options[k].vehicle_backs[l];
							back_2.range = options[k].vehicleRanges[l];

							std::vector<ImagePoint> pointsInRange2 = getPointsInRectangle(allPoints, back_2.range);
							pointsInVehicleRangeOfBacks[back_2] = pointsInRange2;
							numInVehicleDistance[options[k].vehicle_backs[l].first] = pointsInVehicleRangeOfBacks[back_2].size();
							numInVehicleDistance[options[k].vehicle_backs[l].second] = pointsInVehicleRangeOfBacks[back_2].size();

							backPointsMapping[options[k].vehicle_backs[l].first] = back_2;
							backPointsMapping[options[k].vehicle_backs[l].second] = back_2;

							matchedOptions.push_back(i);
							matchedOptions.push_back(j);
							matched = true;
							break;
						}
					}
				}
				if (matched) {
					break;
				}
			}
			if (matched) {
				break;
			}
		}
	}
}


bool VehicleMapping::isInRange(const std::vector<cv::Point2f> &contour, const ImagePoint &point) {
	double res = cv::pointPolygonTest(contour, point, true);
	return (res > 0) || (std::abs(res) < GlobalDataHelper::getInstance().getLEDConst().TOLERANCE_RANGE);
}


void VehicleMapping::calculateOption(const ImagePoint &point, Option &option, std::map<ImagePoint, bool> &marked, const Geometrie &geometrie, const std::vector<ImagePoint> &inRange) {
	std::vector<std::pair<ImagePoint, ImagePoint>> vehicle_backs;
	std::vector<std::vector<cv::Point2f>> vehicle_ranges;
	std::vector<ImagePoint> ends;

	std::vector<ImagePoint> others;

	for (ImagePoint p : inRange) {
		if (p != point) {
			others.push_back(p);
		}
		marked[p] = true;
	}

	for (size_t i = 0; i < others.size(); i++) {
		std::vector<cv::Point2f> contour;
		calculateVehicleRange(point, others[i], geometrie, contour);

		vehicle_backs.push_back(std::pair<ImagePoint, ImagePoint>(point, others[i]));
		vehicle_ranges.push_back(contour);
		ends.push_back(others[(i + 1) % 2]);
	}

	option.ends = ends;
	option.vehicle_backs = vehicle_backs;
	option.vehicleRanges = vehicle_ranges;
}

void VehicleMapping::calculateVehicleRange(const ImagePoint &point1, const ImagePoint &point2, const Geometrie &geometrie, std::vector<cv::Point2f> &contour) {
	double b = sqrt(ImageProcessingHelper::getInstance().getLongestDistance(geometrie, point1.x, point1.y)) + 5;
	double h = sqrt(ImageProcessingHelper::getInstance().getShortDistance(geometrie, point1.x, point1.y));

	std::vector<ImagePoint> points = { point1, point2 };
	ImagePoint center = ImageProcessingHelper::getInstance().calculateCenter(points);

	double angle = 0;

	if (point1.x != point2.x) {
		//function
		double slope = (double)(point1.y - point2.y) / (point1.x - point2.x);
		angle = atan(-1 / slope) / M_PI * 180;
	}

	cv::RotatedRect rectangle = cv::RotatedRect(center, cv::Size2f(2 * b, h), angle);

	cv::Point2f corners[4];
	rectangle.points(corners);

	cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
	contour = std::vector<cv::Point2f>(corners, lastItemPointer);
}


void VehicleMapping::calculateVehicleBack(
	std::map<ImagePoint, bool> &marked,
	const ImagePoint &point,
	const std::vector<ImagePoint> &allPoints,
	Geometrie &geometrie) {

	ImagePoint other = pointsInBackDistance[point][0];

	marked[point] = true;
	marked[other] = true;

	std::vector<cv::Point2f> contour;
	calculateVehicleRange(point, other, geometrie, contour);

	VehicleBack back;
	back.range = contour;
	back.position = std::pair<ImagePoint, ImagePoint>(point, other);

	
	std::vector<ImagePoint> pointsInRange = getPointsInRectangle(allPoints, back.range);
	if (pointsInRange.size() < geometrie.getNumberOfPositionPoints()) {
		//no back
		return;
	}
	pointsInVehicleRangeOfBacks[back] = pointsInRange;
	numInVehicleDistance[point] = pointsInVehicleRangeOfBacks[back].size();
	numInVehicleDistance[other] = pointsInVehicleRangeOfBacks[back].size();

	backPointsMapping[point] = back;
	backPointsMapping[other] = back;
}
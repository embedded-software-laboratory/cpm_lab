#include "ImageProcessingHelper.h"

double ImageProcessingHelper::calculateDistance(const ImagePoint &point1, const ImagePoint &point2) {
	double powX = pow(point1.x - point2.x, 2);
	double powY = pow(point1.y - point2.y, 2);

	return powX + powY;
}

double ImageProcessingHelper::calculateDistance(const WorldPoint &point1, const WorldPoint &point2) {
	double powX = pow(point1.x - point2.x, 2);
	double powY = pow(point1.y - point2.y, 2);
	double powZ = pow(point1.z - point2.z, 2);

	return powX + powY + powZ;
}


double ImageProcessingHelper::getSumDistance(const std::vector<ImagePoint> &points, const ImagePoint &point) {
	double sum = 0;
	for (ImagePoint candidate : points) {
		sum += calculateDistance(candidate, point);
	}
	return sum;
}

double ImageProcessingHelper::getLongestDistance(const Geometrie &geometrie, const int x, const int y) {

	WorldPoint offset = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(ImagePoint(x, y), GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);

	std::vector<ImagePoint> allPoints;
	for (WorldPoint point : geometrie.positionPoints) {
		allPoints.push_back(
			GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(
			WorldPoint(point.x + offset.x, point.y + offset.y, point.z)));
	}

	double distance = 0;
	for (std::vector<ImagePoint>::iterator it1 = allPoints.begin(); it1 != allPoints.end(); it1++) {
		for (std::vector<ImagePoint>::iterator it2 = next(it1, 1); it2 != allPoints.end(); it2++) {
			double dis = ImageProcessingHelper::getInstance().calculateDistance(*it1, *it2);
			if (dis > distance) {
				distance = dis;
			}
		}
	}
	return GlobalDataHelper::getInstance().getLEDConst().TOLERANCE_LONGEST * distance;
}

double ImageProcessingHelper::getShortDistance(const Geometrie &geometrie, int x, int y) {

	WorldPoint offset = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(ImagePoint(x, y), GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);

	std::vector<ImagePoint> allPoints;
	for (WorldPoint point : geometrie.positionPoints) {
		allPoints.push_back(
			GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(
			WorldPoint(point.x + offset.x, point.y + offset.y, point.z)));
	}

	double distance = DBL_MAX;
	for (std::vector<ImagePoint>::iterator it1 = allPoints.begin(); it1 != allPoints.end(); it1++) {
		for (std::vector<ImagePoint>::iterator it2 = next(it1, 1); it2 != allPoints.end(); it2++) {
			double dis = ImageProcessingHelper::getInstance().calculateDistance(*it1, *it2);
            if (dis < distance) {
				distance = dis;
			}
		}
	}
	return distance;
}

double ImageProcessingHelper::getMaximalPossibleDistance(const ImagePoint &start, const Time_Stamp &begin, const Time_Stamp &end) {
	
	double time_in_s = (end - begin) / 1000.0;
	double max_dis_in_m = time_in_s * (20 * 1000 / 3600);

	WorldPoint startWP = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(start, GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);

	WorldPoint radius1(startWP.x + max_dis_in_m, startWP.y, startWP.z);
	WorldPoint radius2(startWP.x - max_dis_in_m, startWP.y, startWP.z);
	WorldPoint radius3(startWP.x, startWP.y + max_dis_in_m, startWP.z);
	WorldPoint radius4(startWP.x, startWP.y - max_dis_in_m, startWP.z);

	std::vector<WorldPoint> radiusWP = { radius1, radius2, radius3, radius4 };
	double max_possible_pixel_distance = 0;

	for (WorldPoint radius : radiusWP) {
		ImagePoint radiusIP = GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(radius);

		double dis = calculateDistance(radiusIP, start);
		if (dis > max_possible_pixel_distance) {
			max_possible_pixel_distance = dis;
		}
	}

	return max_possible_pixel_distance;
}

std::vector<ImagePoint> ImageProcessingHelper::getPointsInDistance(const ImagePoint &point, const double distance, const std::vector<ImagePoint> &allPoints) {

	std::vector<ImagePoint> pointsInDistance;

	for (ImagePoint candidate : allPoints) {

		if (calculateDistance(point, candidate) < distance) {
			pointsInDistance.emplace_back(candidate);
		}
	}

	std::sort(pointsInDistance.begin(), pointsInDistance.end());

	return pointsInDistance;
}

std::vector<ImagePoint> ImageProcessingHelper::getPointsInDistance(const ImagePoint &point, const double distance, const std::vector<ImagePoint> &allPoints, double tolerance_lower, double tolerance_upper) {

	std::vector<ImagePoint> pointsInDistance;

	double distance_root = std::sqrt(distance);
	double lower = pow(distance_root - tolerance_lower, 2);
	double upper = pow(distance_root + tolerance_upper, 2);

	for (ImagePoint candidate : allPoints) {

		double dis = calculateDistance(point, candidate);

		if ((lower < dis) && (dis < upper)) {
			pointsInDistance.emplace_back(candidate);
		}
	}

	std::sort(pointsInDistance.begin(), pointsInDistance.end());

	return pointsInDistance;
}

std::vector<double> ImageProcessingHelper::getSortedDistancesInGeometry(const ImagePoint &point, const Geometrie &geometrie) {
	WorldPoint offset = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(point, GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);

	std::vector<ImagePoint> allPoints;
	for (WorldPoint point : geometrie.positionPoints) {
		allPoints.push_back(
			GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(
			WorldPoint(point.x + offset.x, point.y + offset.y, point.z)));
	}
	for (WorldPoint point : geometrie.identificationPoints) {
		allPoints.push_back(
			GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(
			WorldPoint(point.x + offset.x, point.y + offset.y, point.z)));
	}


	std::vector<double> distances;
	for (std::vector<ImagePoint>::iterator it1 = allPoints.begin(); it1 != allPoints.end(); it1++) {
		for (std::vector<ImagePoint>::iterator it2 = next(it1, 1); it2 != allPoints.end(); it2++) {
			double dis = ImageProcessingHelper::getInstance().calculateDistance(*it1, *it2);
			distances.push_back(dis);
		}
	}
	std::sort(distances.begin(), distances.end());
	return distances;
}


ImagePoint ImageProcessingHelper::calculateCenter(const std::vector<ImagePoint> &points) {
	std::vector<int> setX;
	std::vector<int> setY;

	for (ImagePoint point : points) {
		setX.push_back(point.x);
		setY.push_back(point.y);
	}

	std::sort(setX.begin(), setX.end());
	std::sort(setY.begin(), setY.end());

	int x = setX.front() + ((setX.back() - setX.front()) / 2);
	int y = setY.front() + ((setY.back() - setY.front()) / 2);

	return ImagePoint(x, y);
}

WorldPoint ImageProcessingHelper::calculateCenter(const std::vector<WorldPoint> &region) {

	std::vector<double> setX;
	std::vector<double> setY;
	std::vector<double> setZ;

	for (WorldPoint point : region) {
		setX.push_back(point.x);
		setY.push_back(point.y);
		setZ.push_back(point.z);
	}

	std::sort(setX.begin(), setX.end());
	std::sort(setY.begin(), setY.end());
	std::sort(setZ.begin(), setZ.end());

	double x = setX.front() + ((setX.back() - setX.front()) / 2);
	double y = setY.front() + ((setY.back() - setY.front()) / 2);
	double z = setZ.front() + ((setZ.back() - setZ.front()) / 2);

	return WorldPoint(x, y, z);
}


double ImageProcessingHelper::calculateOrientation(const ImagePoint &sideStart, const ImagePoint &sideEnd, const ImagePoint &thirdPoint, double height) {

	WorldPoint start = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(sideStart, height);
	WorldPoint end = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(sideEnd, height);

	return atan2(end.y - start.y, end.x - start.x);
}

double ImageProcessingHelper::getDifference(double angle1, double angle2) {
	if (angle1 * angle2 >= 0) {
		//both positive or both negative
		return std::abs(angle1 - angle2);
	}

	double sum = std::abs(angle1) + std::abs(angle2);
	if (sum > M_PI) {
		return 2 * M_PI - sum;
	}
	return sum;
}


void ImageProcessingHelper::sortPointsForPositioning(const std::vector<ImagePoint> &pointsForPositioning, WorldPoint &sideStart, WorldPoint &sideEnd, WorldPoint &thirdPoint) {
	WorldPoint zerost = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(pointsForPositioning[0], GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);
	WorldPoint first = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(pointsForPositioning[1], GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);
	WorldPoint second = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(pointsForPositioning[2], GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);

	double distance01 = ImageProcessingHelper::getInstance().calculateDistance(zerost, first);
	double distance02 = ImageProcessingHelper::getInstance().calculateDistance(zerost, second);
	double distance12 = ImageProcessingHelper::getInstance().calculateDistance(first, second);

	if (distance01 < distance02) {
		if (distance01 < distance12) {
			//distance01 is smallest
			//zero and first are the pair
			sideEnd = second;
			calculateStartAndThird(zerost, first, sideEnd, sideStart, thirdPoint);
		}
		else {
			//distance12 is smallest
			//first and second are the pair
			sideEnd = zerost;
			calculateStartAndThird(first, second, sideEnd, sideStart, thirdPoint);
		}
	}
	else {
		if (distance02 < distance12) {
			//distance02 is smallest
			//zero and second are the pair
			sideEnd = first;
			calculateStartAndThird(zerost, second, sideEnd, sideStart, thirdPoint);
		}
		else {
			//distance12 is smallest
			//first and second are the pair
			sideEnd = zerost;
			calculateStartAndThird(first, second, sideEnd, sideStart, thirdPoint);
		}
	}

}

void ImageProcessingHelper::calculateStartAndThird(const WorldPoint &candidate1, const WorldPoint &candidate2, WorldPoint &sideEnd, WorldPoint &sideStart, WorldPoint &thirdPoint) {
	if (candidate1.y > sideEnd.y) {
		if (candidate2.y > sideEnd.y) {
			//both bigger: start is the one with the smaller x value
			if (std::abs(candidate1.x - candidate2.x) < 0.01) {
				//nearly the same
				if (candidate1.x > sideEnd.x) {
					if (candidate1.y > candidate2.y) {
						sideStart = candidate1;
						thirdPoint = candidate2;
						return;
					}
					sideStart = candidate2;
					thirdPoint = candidate1;
					return;
				}
				if (candidate1.y > candidate2.y) {
					sideStart = candidate2;
					thirdPoint = candidate1;
					return;
				}
				sideStart = candidate1;
				thirdPoint = candidate2;
				return;

			}

			if (candidate1.x > candidate2.x) {
				sideStart = candidate2;
				thirdPoint = candidate1;
				return;
			}
			sideStart = candidate1;
			thirdPoint = candidate2;
			return;
		}
		//one smaller one bigger: do at the end
	}
	else {
		if (candidate2.y < sideEnd.y) {
			//both smaller: start is the one with the bigger x value

			if (std::abs(candidate1.x - candidate2.x) < 0.01) {
				//nearlx the same
				if (candidate1.x > sideEnd.x) {
					if (candidate1.y > candidate2.y) {
						sideStart = candidate1;
						thirdPoint = candidate2;
						return;
					}
					sideStart = candidate2;
					thirdPoint = candidate1;
					return;
				}
				if (candidate1.y > candidate2.y) {
					sideStart = candidate2;
					thirdPoint = candidate1;
					return;
				}
				sideStart = candidate1;
				thirdPoint = candidate2;
				return;
			}


			if (candidate1.x < candidate2.x) {
				sideStart = candidate2;
				thirdPoint = candidate1;
				return;
			}
			sideStart = candidate1;
			thirdPoint = candidate2;
			return;
		}
		//one smaller one bigger: do at the end
	}

	//one value is smaller one is bigger
	if (candidate1.x < sideEnd.x) {
		//the x values of the candidates are smaller
		//start value is the one with the smaller y
		if (candidate1.y > candidate2.y) {
			sideStart = candidate2;
			thirdPoint = candidate1;
			return;
		}
		sideStart = candidate1;
		thirdPoint = candidate2;
		return;
	}
	//the x values of the candidates are bigger
	//start value is the one with the bigger y
	if (candidate1.y < candidate2.y) {
		sideStart = candidate2;
		thirdPoint = candidate1;
		return;
	}
	sideStart = candidate1;
	thirdPoint = candidate2;
}


void ImageProcessingHelper::calculateMinMaxMeanStdDev(const std::vector<double> &values, double &min, double &max, double &mean, double &stdDev) {
	min = *std::min_element(values.begin(), values.end());
	max = *std::max_element(values.begin(), values.end());
	double sum = std::accumulate(values.begin(), values.end(), 0.0);
	mean = sum / (double)values.size();

	double variance = 0.0;
	for (size_t i = 0; i < values.size(); i++) {
		variance += std::pow(values[i] - mean, 2);
	}
	variance = variance / values.size();
	stdDev = std::sqrt(variance);
}


static inline double Lerp(double v0, double v1, double t)
{
	return (1 - t)*v0 + t*v1;
}


static inline double getQuantil(std::vector<double> &data, double p) {
	double poi = Lerp(-0.5, data.size() - 0.5, p);

	size_t left = std::max(int64_t(std::floor(poi)), int64_t(0));
	size_t right = std::min(int64_t(std::ceil(poi)), int64_t(data.size() - 1));

	double datLeft = data.at(left);
	double datRight = data.at(right);

	return Lerp(datLeft, datRight, poi - left);
}

void ImageProcessingHelper::calculateBoxplot(const std::vector<double> &values, double &median, double &upperQuantil, double &lowerQuantil, double &upperWhisker, double &lowerWhisker) {

	if (values.empty()) {
		return;
	}

	std::vector<double> data = values;
	std::sort(data.begin(), data.end());


	upperQuantil = getQuantil(data, 0.75);
	lowerQuantil = getQuantil(data, 0.25);
	median = getQuantil(data, 0.5);

	double iqr = upperQuantil - lowerQuantil;

	double upperFence = upperQuantil + 1.5 * iqr;
	double lowerFence = lowerQuantil - 1.5 * iqr;

	if (upperFence > data.back()) {
		upperWhisker = data.back();
	}
	else {
		for (int i = data.size() - 1; i >= 0; i--) {
			if (data[i] < upperFence) {
				upperWhisker = data[i];
				break;
			}
		}
	}

	if (lowerFence < data.front()) {
		lowerWhisker = data.front();
	}
	else {
		for (size_t i = 0; i < data.size(); i++) {
			if (data[i] > lowerFence) {
				lowerWhisker = data[i];
				break;
			}
		}
	}

}

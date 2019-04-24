#include "TargetLED.h"


TargetLED::TargetLED(const int initial_negative_ID)
{
	negative_ID = initial_negative_ID;
}


TargetLED::~TargetLED()
{
}


bool TargetLED::addNewImage(const TimedPoints &points) {

	struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

	//calculatePositionAndOrientation of points
	Location new_location;
	calculatePositionAndOrientation(points, new_location);

	//calculateID with this points
	int new_id = calculateIdentification(points);

	//check calculation
	bool plausible = isPlausible(new_id, new_location, points.timestamp);

	if (!plausible) {
        clock_gettime(CLOCK_REALTIME, &t);
        uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
		GlobalDataHelper::getInstance().addDurationTarget(t2 - t1);
		return false;
	}

	//if plausible set points to last points; set timepoints, set counters return true
	adaptCounters(points);
	targetPointsFromLastImage = std::make_shared<TimedPoints>(points);
	timestamp = points.timestamp;

	if (!initial_off && !initial_on) {
		id = new_id;
	}

	location = new_location;
	locationHistory.push_back(location.position);
	historyTimePoints.push_back(timestamp);
	if (locationHistory.size() > num_of_interpolation_points) {
		locationHistory.erase(locationHistory.begin());
		historyTimePoints.erase(historyTimePoints.begin());
	}

	center = ImageProcessingHelper::getInstance().calculateCenter(points.points);

    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
	GlobalDataHelper::getInstance().addDurationTarget(t2 - t1);

	return true;
}

//====================== identification ==============================================================

void TargetLED::adaptCounters(const TimedPoints &points) {
	//check if following image
	if (counter != 0 && points.timestamp - timestamp > 22000000) {
		std::cout << "not following image " << points.timestamp - timestamp << std::endl;
		//distance more than 20ms -> one image is missing
		bool ledState = points.points.size() != GlobalDataHelper::getInstance().getGeometrie().getNumberOfPositionPoints();

		if (ledState != ledOn) {
			//different from state before coverage
			if (ledOn && !onCounters.empty()) {
				onCounters.erase(onCounters.begin());
			}
			else if (!ledOn && !offCounters.empty()) {
				offCounters.erase(offCounters.begin());
			}
		}
		removeOnChange = true;
	}
	else if (counter == 0) {
		ledOn = points.points.size() != GlobalDataHelper::getInstance().getGeometrie().getNumberOfPositionPoints();
		counter++;
	}
	else {
		onCounters = onState;
		offCounters = offState;
		counter = counter_copy;
		ledOn = ledOn_copy;
		removeOnChange = removeOnChange_copy;
		initial_on = initial_on_copy;
		initial_off = initial_off_copy;
	}
}

void TargetLED::increaseCounters(const std::vector<ImagePoint> &points) {
	if (points.size() == GlobalDataHelper::getInstance().getGeometrie().getNumberOfPositionPoints()) {
		//LED off
		if (ledOn_copy) {
			if (!removeOnChange_copy) {
				onState.emplace_back(counter_copy);
			}
			removeOnChange_copy = false;
			counter_copy = 1;
			ledOn_copy = false;
		}
		else {
			counter_copy++;
		}
	}
	else {
		//LED on
		if (ledOn_copy) {
			counter_copy++;
		}
		else {
			if (!removeOnChange_copy) {
				offState.emplace_back(counter_copy);
			}
			removeOnChange_copy = false;
			counter_copy = 1;
			ledOn_copy = true;
		}
	}
}

void TargetLED::limitVectors() {
	if (onState.size() > MAX_IMAGES_PER_TARGET) {
		onState.erase(onState.begin());
	}
	if (offState.size() > MAX_IMAGES_PER_TARGET) {
		offState.erase(offState.begin());
	}
}

double TargetLED::getTime(const std::vector<size_t> &counters) {
	size_t sum = 0;
	for (size_t num : counters) {
		sum += num;
	}

	return (double)sum / counters.size();
}

IdentificationCharacteristic TargetLED::getCharacteristic(double onFreqReal, double offFreqReal) {

	Interval onInt;
	Interval offInt;

	bool finished = false;
	for (Interval interval : GlobalDataHelper::getInstance().getPossibleIntervalsLED()) {
		if (interval.in(onFreqReal)){
			onInt = interval;
			if (finished) {
				break;
			}
			finished = true;
		}
		if (interval.in(offFreqReal)){
			offInt = interval;
			if (finished) {
				break;
			}
			finished = true;
		}
	}
	return IdentificationCharacteristic(onInt, offInt);
}

int TargetLED::calculateIdentification(const TimedPoints &points) {

	if ((counter == 0) || (points.timestamp - timestamp > 20000000)) {
		//not following up image or first iteration -> no conclusion about id possible -> so take old id
        return id;
	}

	//copy state
	onState = onCounters;
	offState = offCounters;
	counter_copy = counter;
	ledOn_copy = ledOn;
	removeOnChange_copy = removeOnChange;
	initial_on_copy = initial_on;
	initial_off_copy = initial_off;

	//add points to counters
	increaseCounters(points.points);
	if (initial_on_copy && onState.size() > 1) {
		onState.erase(onState.begin());
		initial_on_copy = false;
	}
	if (initial_off_copy && offState.size() > 1) {
		offState.erase(offState.begin());
		initial_off_copy = false;
	}
	limitVectors();

	double onTime = getTime(onState);
	double offTime = getTime(offState);

	double onFreqReal = GlobalDataHelper::getInstance().getLEDConst().CAMERA_FREQUENCY / onTime;
	double offFreqReal = GlobalDataHelper::getInstance().getLEDConst().CAMERA_FREQUENCY / offTime;

	IdentificationCharacteristic characteristic = getCharacteristic(onFreqReal, offFreqReal);

	std::map<IdentificationCharacteristic, int> identificationMap;
	GlobalDataHelper::getInstance().getIdentificationMap(identificationMap);

	auto it = identificationMap.find(characteristic);
	if (it == identificationMap.end()) {
		return negative_ID;
	}

	return it->second;
}


//====================== location ====================================================================

void TargetLED::calculatePositionAndOrientation(const TimedPoints &points, Location &location) {
	std::vector<ImagePoint> pointsForPositioning;
	getPointsForPositioning(points, pointsForPositioning);

	if (pointsForPositioning.empty()) {
		return;
	}

	ImageProcessingHelper::getInstance().sortPointsForPositioning(pointsForPositioning, sideStart, sideEnd, thirdPoint);

	
	//orientation
	location.orientation = calculateOrientation(points.timestamp, location);
	if (location.orientation > M_PI) {
		location.orientation = location.orientation - (2 * M_PI);
	}
	else if (location.orientation < -M_PI) {
		location.orientation = location.orientation + (2 * M_PI);
	}

	//position
	std::vector<WorldPoint> axis = { sideStart, thirdPoint };
	WorldPoint worldCenter = ImageProcessingHelper::getInstance().calculateCenter(axis);
	location.position = calculatePositionWithOffset(worldCenter, location.orientation);
}

void TargetLED::getPointsForPositioning(const TimedPoints &points, std::vector<ImagePoint> &pointsForPositioning) {

	//only three points in last image -> no identification point
	if (points.points.size() == GlobalDataHelper::getInstance().getGeometrie().getNumberOfPositionPoints()) {
		pointsForPositioning = points.points;
		return;
	}

	//get point with smallest sum of distances to all other points -> identification point
	//take all other points
	ImagePoint identificationPoint = points.points.front();
	double min_dis_sum = ImageProcessingHelper::getInstance().getSumDistance(points.points, identificationPoint);
	for (std::vector<ImagePoint>::const_iterator point = next(points.points.begin(), 1); point != points.points.end(); point++) {
		double dis = ImageProcessingHelper::getInstance().getSumDistance(points.points, *point);
		if (dis < min_dis_sum) {
			min_dis_sum = dis;
			identificationPoint = *point;
		}
	}

	for (ImagePoint point : points.points) {
		if (point != identificationPoint) {
			pointsForPositioning.push_back(point);
		}
	}
}

WorldPoint TargetLED::calculatePositionWithOffset(const WorldPoint &position, double orientation) {
	WorldPoint offset = GlobalDataHelper::getInstance().getGeometrie().offsetMidPoint;

	double beta = M_PI / 2 + orientation;
	double x_offset = (-offset.x) * cos(orientation) + (-offset.y) * cos(beta);
	double y_offset = (-offset.x) * sin(orientation) + (-offset.y) * sin(beta);

	return WorldPoint(position.x + x_offset, position.y + y_offset, position.z);
}


double TargetLED::calculateOrientation(const Time_Stamp &timestamp, const Location &location) {

	double ori_side = atan2(sideEnd.y - sideStart.y, sideEnd.x - sideStart.x) + GlobalDataHelper::getInstance().getLEDConst().OFFSET_RIGHT;
    double ori_back = atan2(sideStart.y - thirdPoint.y, sideStart.x - thirdPoint.x)+ M_PI /2;
    double ori_diag = atan2(sideEnd.y - thirdPoint.y, sideEnd.x - thirdPoint.x) + GlobalDataHelper::getInstance().getLEDConst().OFFSET_LEFT;

    if(ori_side > M_PI) {
        ori_side -= 2*M_PI;
    }
    if(ori_back > M_PI) {
        ori_back -= 2*M_PI;
    }
    if(ori_diag > M_PI) {
        ori_diag -= 2*M_PI;
    }

  //  std::cout << ori_side << ", " << ori_back << ", " << ori_diag << std::endl;
  //  std::cout << sideStart << ", " << sideEnd << ", " << thirdPoint << std::endl;

	return calculateOrientationAsMedian(ori_side, ori_back, ori_diag);

	//return calculateOrientationAsAverage(ori_side, ori_back, ori_diag);
	//return calculateOrientationWithHistory(timestamp, location);
}


double TargetLED::calculateOrientationAsAverage(const double ori_side, const double ori_back, const double ori_diag) {
	if ((ori_side * ori_back < 0 || ori_back * ori_diag < 0) && std::abs(ori_side) > M_PI / 2) {

		std::vector<double> orientations = { ori_side, ori_back, ori_diag };
		double sum = 0;
		for (double ori : orientations) {
			if (ori > 0) {
				sum += ori;
			}
			else {
				sum += ori;
				sum += 2 * M_PI;
			}
		}

		double average = sum / 3;
		if (average > M_PI) {
			return average - 2 * M_PI;
		}
		return average;
	}

	return (ori_side + ori_back + ori_diag) / 3;
}

double TargetLED::calculateOrientationAsMedian(const double ori_side, const double ori_back, const double ori_diag) {
	if ((ori_side * ori_back < 0 || ori_back * ori_diag < 0) && std::abs(ori_side) > M_PI / 2) {

		std::vector<double> orientations = { ori_side, ori_back, ori_diag };
		std::vector<double> orientations_Positive;
		for (double ori : orientations) {
			if (ori > 0) {
				orientations_Positive.push_back(ori);
			}
			else {
				orientations_Positive.push_back(ori + 2 * M_PI);
			}
		}

		std::sort(orientations_Positive.begin(), orientations_Positive.end());
		double median = orientations_Positive[1];

		if (median > M_PI) {
			return median - 2 * M_PI;
		}
		return median;
	}

	std::vector<double> orientations = { ori_side, ori_back, ori_diag };
	std::sort(orientations.begin(), orientations.end());

	return orientations[1];
}


//===================== Plausibility =================================================================

bool TargetLED::isPlausible(const int calculated_id, const Location &calculated_location, const Time_Stamp &calculated_timestamp) {

	if (id == 0) {
		//inital
		return true;
	}
	
	if (isIDPlausible(calculated_id) && 
		isPositionPlausible(calculated_location, calculated_timestamp) && 
		isOrientationPlausible(calculated_location, calculated_timestamp)) {
		return true;
	}

	return false;
}

bool TargetLED::isIDPlausible(const int calculated_id) {
	if (id < 0) {
		return true;
	}

	if (initial_on || initial_off) {
		//not enough values for reference
		return true;
	}

	if (id == calculated_id) {
		return true;
	}
	return false;
}

bool TargetLED::isPositionPlausible(const Location &calculated_location, const Time_Stamp &calculated_timestamp) {
	//time in s
	double elapsedTime = (calculated_timestamp - timestamp) / 1000.0;
	
	//speed in m/s
	double max_speed = 20 / 3600.0 * 1000;

	//max distance in m
	double max_distance = max_speed * elapsedTime;

	//distance in m
	double distance = ImageProcessingHelper::getInstance().calculateDistance(calculated_location.position, location.position);

	return distance <= pow(max_distance,2);
}

bool TargetLED::isOrientationPlausible(const Location &calculated_location, const Time_Stamp &calculated_timestamp) {

	//time in s
	double elapsedTime = (calculated_timestamp - timestamp) / 1000.0;

	//maximale seitliche Beschleuinigung in m/s^2
	double a_max = 12;

	//konstant in 1/m
	double kappa_max = 5;

	//maximal angle in the elapsed time
	double angle_max = elapsedTime * sqrt(a_max * kappa_max);

	//difference to current angle
	double angle_diff = ImageProcessingHelper::getInstance().getDifference(location.orientation, calculated_location.orientation);

	return angle_diff  <= angle_max;
}

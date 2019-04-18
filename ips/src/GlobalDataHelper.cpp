#include "GlobalDataHelper.h"


void GlobalDataHelper::setStartTime(const uint64_t &startTime, const uint64_t startTick) {
	this->startTime = startTime - startTick;
}

uint64_t GlobalDataHelper::getStartTime() {
	return startTime;
}


void GlobalDataHelper::setCameraParameters(const std::shared_ptr<CameraParameter> cameraParameter) {
	this->cameraParameter = cameraParameter;
}

std::shared_ptr<CameraParameter> GlobalDataHelper::getCameraParameters() {
	return cameraParameter;
}


void GlobalDataHelper::setGeometrie(const Geometrie &geometrie) {
	this->geometrie = geometrie;

	//(RH.y-RV.y) /(RH.x -RV.x)
	ledConst.OFFSET_RIGHT = -asin((geometrie.positionPoints[1].y - geometrie.positionPoints[2].y) / (geometrie.positionPoints[1].x - geometrie.positionPoints[2].x));
	//(RV.x-LH.x)/(RV.y-LH.y)
	ledConst.OFFSET_LEFT = M_PI / 2 + atan((geometrie.positionPoints[2].x - geometrie.positionPoints[0].x) / (geometrie.positionPoints[2].y - geometrie.positionPoints[0].y));
}

Geometrie GlobalDataHelper::getGeometrie() {
	return geometrie;
}


void GlobalDataHelper::setSlopeOfReferenceLine(const WorldPoint &start, const WorldPoint &end) {

	assert(start.z == end.z);
	slopeOfReferenceLine = ((double)end.y - start.y) / (end.x - start.x);
}

double GlobalDataHelper::getSlopeOfReferenceLine() {
	return slopeOfReferenceLine;
}


void GlobalDataHelper::setGlobalConst(const double height, const double roof_height, const int exposure_time) {
	globalConst.HEIGHT_VEHICLE = height;
	globalConst.HEIGHT_VEHICLE_ROOF = roof_height;
	globalConst.EXPOSURE_TIME = exposure_time;
}

GlobalConstants GlobalDataHelper::getGlobalConst() {
	return globalConst;
}


void GlobalDataHelper::setLEDConst(const double cameraFrequency) {
	ledConst.CAMERA_FREQUENCY = cameraFrequency;
}

void GlobalDataHelper::setToleranceShortest(const double tolerance_shortest_low, const double tolerance_shortest_high) {
	ledConst.TOLERANCE_SHORTEST_LOW = tolerance_shortest_low;
	ledConst.TOLERANCE_SHORTEST_HIGH = tolerance_shortest_high;
}

void GlobalDataHelper::setToleranceLongest(const double tolerance_longest) {
	ledConst.TOLERANCE_LONGEST = 1 + tolerance_longest;
}

void GlobalDataHelper::setToleranceRange(const double tolerance_range) {
	ledConst.TOLERANCE_RANGE = tolerance_range;
}

void GlobalDataHelper::setToleranceGeometry(const double tolerance_geometry) {
	ledConst.TOLERANCE_GEOMETRY_DISTANCES = tolerance_geometry;
}

LEDConstants GlobalDataHelper::getLEDConst() {
	return ledConst;
}


void GlobalDataHelper::addPossibleIntervalLED(const Interval &interval) {
	if (std::find(intervalsLED.begin(), intervalsLED.end(), interval) != intervalsLED.end()) {
		//check if already inserted
		return;
	}
	//check if not conflicting
	for (Interval possibleInt : intervalsLED) {
		if ((possibleInt.min < interval.max) && (interval.min < possibleInt.max)) {
			throw std::runtime_error("Intersecting Intervals!!!");
		}

		
	}

	intervalsLED.emplace_back(interval);
}

std::vector<Interval> GlobalDataHelper::getPossibleIntervalsLED() {
	return intervalsLED;
}



void GlobalDataHelper::setRealSituations(const std::vector<Situation> &realSituations) {
	this->realSituations = realSituations;
}

void GlobalDataHelper::getRealSituations(std::vector<Situation> &realSituations) {
	realSituations = this->realSituations;
}


void GlobalDataHelper::setIdentificationMap(const std::map<IdentificationCharacteristic, int> &vehicles) {
	identificationMap = vehicles;
}

void GlobalDataHelper::getIdentificationMap(std::map<IdentificationCharacteristic, int> &vehicles) {
	vehicles = identificationMap;
}

void GlobalDataHelper::evaluateDurations(std::ofstream &outputFile) {

	double min, max, mean, stdDev;

	if (durationsImageHolder.size() > 2) {

        durationsImageHolder.erase(durationsImageHolder.begin());
        durationsImageHolder.pop_back();

		outputFile << "\nImageHolder:\n";
		ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(durationsImageHolder, min, max, mean, stdDev);

		outputFile << "   Min = " << min << "\n";
        outputFile << "   Max = " << max << "\n";
        outputFile << "   Mean= " << mean << "\n";
        outputFile << "   Stddev = " << stdDev << "\n";
	}


	if (durationsDetectedObjects.size() > 2) {

        durationsDetectedObjects.erase(durationsDetectedObjects.begin());
        durationsDetectedObjects.pop_back();

		outputFile << "\nDetectedObjects:\n";
		ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(durationsDetectedObjects, min, max, mean, stdDev);

		outputFile << "   Min = " << min << "\n";
        outputFile << "   Max = " << max << "\n";
        outputFile << "   Mean= " << mean << "\n";
        outputFile << "   Stddev = " << stdDev << "\n";
	}

	if (durationsSortedObjects.size() > 2) {

        durationsSortedObjects.erase(durationsSortedObjects.begin());
        durationsSortedObjects.pop_back();

		outputFile << "\nSortedObjects:\n";
		ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(durationsSortedObjects, min, max, mean, stdDev);

		outputFile << "   Min = " << min << "\n";
        outputFile << "   Max = " << max << "\n";
        outputFile << "   Mean= " << mean << "\n";
        outputFile << "   Stddev = " << stdDev << "\n";
	}

/*	if (durationsTargetsEach.size() > 2) {

        durationsTargetsEach.erase(durationsTargetsEach.begin());
        durationsTargetsEach.pop_back();

		std::cout << std::endl << "Targtes Each:" << std::endl;
		ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(durationsTargetsEach, min, max, mean, stdDev);

		std::cout << "   Min = " << min << std::endl;
		std::cout << "   Max = " << max << std::endl;
		std::cout << "   Mean= " << mean << std::endl;
		std::cout << "   Stddev = " << stddev << std::endl;
	}*/

	if (durationsTargets.size() > 2) {

        durationsTargets.erase(durationsTargets.begin());
        durationsTargets.pop_back();

		outputFile << "\nTargets:\n";
		ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(durationsTargets, min, max, mean, stdDev);

		outputFile << "   Min = " << min << "\n";
        outputFile << "   Max = " << max << "\n";
        outputFile << "   Mean= " << mean << "\n";
        outputFile << "   Stddev = " << stdDev << "\n";
	}
}

void GlobalDataHelper::addDurationImageHolder(long long duration) {
	durationsImageHolder.push_back(duration / unit);
    if(durationsImageHolder.size() > 1000) {
        durationsImageHolder.erase(durationsImageHolder.begin());
    }
}

void GlobalDataHelper::addDurationDetectedObjects(long long duration) {
	durationsDetectedObjects.push_back(duration / unit);
     if(durationsDetectedObjects.size() > 1000) {
        durationsDetectedObjects.erase(durationsDetectedObjects.begin());
    }
}

void GlobalDataHelper::addDurationSortedObjects(long long duration) {
	durationsSortedObjects.push_back(duration / unit);
     if(durationsSortedObjects.size() > 1000) {
        durationsSortedObjects.erase(durationsSortedObjects.begin());
    }
}

void GlobalDataHelper::addDurationTarget(long long duration) {
	durationsTargetsEach.push_back(duration / unit);
     if(durationsTargetsEach.size() > 1000) {
        durationsTargetsEach.erase(durationsTargetsEach.begin());
    }
}

void GlobalDataHelper::addDurationTargets(long long duration) {
	durationsTargets.push_back(duration / unit);
     if(durationsTargets.size() > 1000) {
        durationsTargets.erase(durationsTargets.begin());
    }
}

void GlobalDataHelper::cancelProgram() {
	cancel = true;
}

bool GlobalDataHelper::isCanceled() {
	return cancel;
}
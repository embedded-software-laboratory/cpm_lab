#include "CommunicationAPI_LED.h"
#include <fstream>

CommunicationAPI_LED::CommunicationAPI_LED(const std::string &serial_no, const std::string &configFilePath)
	: CommunicationAPI(serial_no, configFilePath)
{
	readConfigFromFile(configFilePath);
}


CommunicationAPI_LED::~CommunicationAPI_LED()
{
}


void CommunicationAPI_LED::readConfigFromFile(const std::string &configFilePath) {
	std::ifstream file(configFilePath);

	if (!file.is_open()) {
		throw std::runtime_error(__FILE__ ": File could not be read! Config File invalid");
	}

	bool camFrequency = false;
	bool vehicleGeo = false;
	bool numThreads = false;
	int idMapping = 0;

	double frequency_low, frequency_high;
	int numIDs, disImages;

	std::string line;
	while (getline(file, line)) {

		if (line.find("\"Camera Frequency\":") == 0) {
			GlobalDataHelper::getInstance().setLEDConst(FileReaderHelper::getInstance().getDoubleFromString(line, 20));
			camFrequency = true;
		}

		
		if (line.find("\"Vehicle Geometrie\":") == 0) {
			Geometrie geometrie;
			FileReaderHelper::getInstance().readGeometrieFromString(line, geometrie);
			std::vector<WorldPoint> backPoints = { geometrie.positionPoints[0], geometrie.positionPoints[1] };
			geometrie.offsetMidPoint = ImageProcessingHelper::getInstance().calculateCenter(backPoints);
			geometrie.offsetMidPoint.z = 0; //z is not relevant for this
			GlobalDataHelper::getInstance().setGeometrie(geometrie);
			vehicleGeo = true;
		}

		if (line.find("\"Number Threads\":") == 0) {
			size_t det = line.find_first_of(",");
			size_t sort = line.find_first_of(",", det + 1);

			numDetectionThreads = FileReaderHelper::getInstance().getIntFromString(line, 18, det);
			numSortingThreads = FileReaderHelper::getInstance().getIntFromString(line, det + 2, sort);
			numRecognizingThreads = FileReaderHelper::getInstance().getIntFromString(line, sort + 2);
			
			numThreads = true;
		}

		if (line.find("\"Camera Frequency Range\":") == 0)  {
			size_t low = line.find_first_of("-");

			frequency_low = FileReaderHelper::getInstance().getIntFromString(line, 26, low);
			frequency_high = FileReaderHelper::getInstance().getIntFromString(line, low + 1);

			idMapping++;
		}


		if (line.find("\"NumberIDs\":") == 0)  {
			numIDs = FileReaderHelper::getInstance().getIntFromString(line, 13);
			idMapping++;
		}


		if (line.find("\"DistanceImages\":") == 0)  {
			disImages = FileReaderHelper::getInstance().getIntFromString(line, 18);
			idMapping++;
		}

		if (line.find("\"Mapping\":") == 0) {

			std::string map;
			size_t pos = 0;
			size_t map_end_pos;

			while (FileReaderHelper::getInstance().findNextMapping(line, pos, map, map_end_pos)) {
				
				size_t end_id = map.find_first_of(",");
				int id = FileReaderHelper::getInstance().getIntFromString(map, 5, end_id);

				if (id == 0) {
					throw std::runtime_error("ID is not allowed to be 0");
				}

				std::string frequency;
				size_t pos_f;
				FileReaderHelper::getInstance().findNextCharPair(map, "(", ")", 0, frequency, pos_f);

				size_t pos_int_end;
				std::string intOn, intOff;
				FileReaderHelper::getInstance().findNextPoint(frequency, 0, intOn, pos_int_end);
				FileReaderHelper::getInstance().findNextPoint(frequency, pos_int_end, intOff, pos_int_end);

				Interval onInt = FileReaderHelper::getInstance().readIntervalFromString(intOn);
				Interval offInt = FileReaderHelper::getInstance().readIntervalFromString(intOff);

				addVehicle(id, IdentificationCharacteristic(onInt, offInt));
				GlobalDataHelper::getInstance().addPossibleIntervalLED(onInt);
				GlobalDataHelper::getInstance().addPossibleIntervalLED(offInt);

				pos = map_end_pos;
			}
			idMapping = 3;
		}

		if (line.find("\"Tolerance longest distance\":") == 0)  {
			GlobalDataHelper::getInstance().setToleranceLongest(FileReaderHelper::getInstance().getDoubleFromString(line, 30));
		}

		if (line.find("\"Tolerance shortest distance\":") == 0)  {

			size_t sep = line.find_first_of(";");

			double low = FileReaderHelper::getInstance().getDoubleFromString(line, 31, sep);
			double high = FileReaderHelper::getInstance().getDoubleFromString(line, sep + 1);

			GlobalDataHelper::getInstance().setToleranceShortest(low, high);
		}

		if (line.find("\"Tolerance in Range\":") == 0)  {
			GlobalDataHelper::getInstance().setToleranceRange(FileReaderHelper::getInstance().getDoubleFromString(line, 22));
		}

		if (line.find("\"Tolerance geoemtrie distance\":") == 0)  {
			GlobalDataHelper::getInstance().setToleranceGeometry(FileReaderHelper::getInstance().getDoubleFromString(line, 32));
		}
	}

	file.close();

	assert(camFrequency & vehicleGeo & numThreads & (idMapping == 3));

	if (vehicles.empty()) {
		calculateFrequencies(numIDs, disImages, frequency_low, frequency_high);
	}
	GlobalDataHelper::getInstance().setIdentificationMap(vehicles);
}

void CommunicationAPI_LED::cleanUp() {
	running = false;

	for (size_t i = 0; i < recognizingThreads.size(); i++) {
		recognizingThreads[i].join();
	}

    std::cout << "join recognizingThreads" << std::endl;

	for (size_t i = 0; i < sortingThreads.size(); i++) {
		sortingThreads[i].join();
	}

    std::cout << "join sortingThreads" << std::endl;

	for (size_t i = 0; i < detectionThreads.size(); i++) {
		detectionThreads[i].join();
	}

    std::cout << "join detectionThreads" << std::endl;
	
	camera->close();

    std::cout << "close camera" << std::endl;
}

bool CommunicationAPI_LED::setCamera() {
	camera.reset(new ImageHolderLED(serial_no));
//	cv::waitKey(100);
	startThreads();
	if (GlobalDataHelper::getInstance().isCanceled()) {
		cleanUp();
		return false;
	}
	return true;
}

bool CommunicationAPI_LED::setCamera(const std::string &filename) {
	camera.reset(new ImageHolderLED(serial_no, filename));
	//cv::waitKey(100);
	startThreads();
	if (GlobalDataHelper::getInstance().isCanceled()) {
		cleanUp();
		return false;
	}
	return true;
}

void CommunicationAPI_LED::startThreads() {
	//start threads processing images
	for (int i = 0; i < numDetectionThreads; i++) {
		detectionThreads.push_back(std::thread([this] { this->detectObjects(); }));
	}

	for (int i = 0; i < numSortingThreads; i++) {
		sortingThreads.push_back(std::thread([this] { this->sortObjects(); }));
	}

	for (int i = 0; i < numRecognizingThreads; i++) {
		recognizingThreads.push_back(std::thread([this] { this->regognizeTargets(); }));
	}

	//wait to calculate the first 30 images
	cv::waitKey(2000);
}

void CommunicationAPI_LED::getCurrentSituation(Situation &situation) {

	while (!refreshed) {
		if (GlobalDataHelper::getInstance().isCanceled()) {
			return;
		}
		//cv::waitKey(1);
	}

	targetLock->lock();

    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

	situation.timestamp = targets.timestamp;

	std::vector<int> removeIds;

	for (std::shared_ptr<Target> target : targets.elements) {
		if (target->getTimeStamp() != targets.timestamp) {
			continue;
		}
		int id = target->getID();
		Location pos;
		target->getLocation(pos);

		if (situation.mapping.find(id) != situation.mapping.end() && id != 0) {
			//id already in mapping - not plausible
			situation.mapping.erase(id);
			removeIds.push_back(id);
			std::cerr << "at least two vehicles with ID " << id << std::endl;
		}

		situation.mapping[id] = pos;
	}
	refreshed = false;

	removeCond cond(removeIds);
	targets.elements.remove_if(cond);

	clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
	GlobalDataHelper::getInstance().addDurationTargets(t2 - t1);

	targetLock->unlock();
}

void CommunicationAPI_LED::detectObjects() {

	while (running && !GlobalDataHelper::getInstance().isCanceled()) {
		std::shared_ptr<DetectedObjects> objects;
		camera->detectObjectsInCurrentImages(objects);
		detectedObjectQueue.enqueue(objects->getID() ,objects);
	}
	
	if (GlobalDataHelper::getInstance().isCanceled()) {
		std::cout << "Canceled Detection Thread" << std::endl;
	}

}

void CommunicationAPI_LED::sortObjects() {

	while (running && !GlobalDataHelper::getInstance().isCanceled()) {

		std::shared_ptr<DetectedObjects> objects = nullptr;
		bool success = false;
		while (!success && running) {
			success = detectedObjectQueue.dequeue(objects);
		}

		if (objects != nullptr) {
			std::shared_ptr<SortedObjects> sortedObjects;
			objects->sortPoints(sortedObjects);
			sortedObjectsQueue.enqueue(objects->getID(), sortedObjects);
		}
	}

	if (GlobalDataHelper::getInstance().isCanceled()) {
		std::cout << "Canceled Sorting Thread" << std::endl;
	}
}

void CommunicationAPI_LED::regognizeTargets() {

	while (running && !GlobalDataHelper::getInstance().isCanceled()) {

		std::shared_ptr<SortedObjects> objects = nullptr;
		bool success = false;
		while (!success && running) {
			success = sortedObjectsQueue.dequeue(objects);
		}

		if (objects != nullptr) {
			targetLock->lock();
			objects->matchToTargets(targets);

            struct timespec t;
            clock_gettime(CLOCK_REALTIME, &t);
            uint64_t timePoint = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

            newSituation.push_back(timePoint);
            situations.push_back(targets.timestamp);
            refreshed = true;
            numImagesTargets++;

			targetLock->unlock();
		}
	}

	if (GlobalDataHelper::getInstance().isCanceled()) {
		std::cout << "Canceled Recognizing Thread" << std::endl;
	}
}

//floor and the cantor pairing always return an integer value only stored as double/float - no lost of data
void CommunicationAPI_LED::calculateFrequencies(int numIDs, int disImages, double frequency_low, double frequency_high) {
	int numFrequences = floor(sqrt(numIDs)) + 1;
	//calculate ids
	for (int i = (disImages / 2) + 1; i < numFrequences * disImages; i = i + disImages) {
		Interval intOn(frequency_low / (i + 1), frequency_high / (i - 1));
		for (int j = (disImages / 2) + 1; j < numFrequences * disImages; j = j + disImages)  {
			Interval intOff(frequency_low / (j + 1), frequency_high / (j - 1));
			IdentificationCharacteristic characteristic(intOn, intOff);
			//cantor pairing function - always returns int
			int id = 0.5 * (i + j) * (i + j + 1) + j;
			addVehicle(id, characteristic);
		}
		GlobalDataHelper::getInstance().addPossibleIntervalLED(intOn);
	}
}

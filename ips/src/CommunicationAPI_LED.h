#pragma once
#include "CommunicationAPI.h"
#include "DetectedObjectsBuffer.h"

#include "ImageHolderLED.h"
#include "SortedObjects.h"
#include "DetectedObjects.h"

class CommunicationAPI_LED :
	public CommunicationAPI
{
public:
	CommunicationAPI_LED(const std::string &serial_no, const std::string &filePath);
	~CommunicationAPI_LED();

	/**
		set camera for the positioning
	*/
	bool setCamera();
	/**
		set camera with filename if CameraFileWrapper is used
	*/
	bool setCamera(const std::string &filename);

	/**
		get current situation of the vehicles
	*/
	void getCurrentSituation(Situation &situation);

	/**
		clean up: join threads
	*/
	void cleanUp();

    std::vector<double> getTimePointDistances() {
        return timePointDistances;
    }


private:

	/**
		camera
	*/
	std::shared_ptr<ImageHolderLED> camera;

	/**
		if running = false the threads are joined
	*/
	bool running = true;

	/**
		despicts whether the situation is updated
	*/
	bool refreshed = false;

	/**
		describe the number of threads
	*/
	int numDetectionThreads = 1;
	int numSortingThreads = 1;
	int numRecognizingThreads = 1;

	/**
		list of targets and their timestamps
	*/
	Targets targets;

	/**
		lock for the targets
	*/
	std::shared_ptr<std::mutex> targetLock = std::make_shared<std::mutex>();

	/**
		queues for the detection and sorting
	*/
	DetectedObjectsBuffer<std::shared_ptr<DetectedObjects>> detectedObjectQueue;
	DetectedObjectsBuffer<std::shared_ptr<SortedObjects>> sortedObjectsQueue;

	/**
		threads
	*/
	std::vector<std::thread> detectionThreads;
	std::vector<std::thread> sortingThreads;
	std::vector<std::thread> recognizingThreads;

    std::vector<Time_Stamp> newSituation;
    std::vector<Time_Stamp> situations;

    int numImagesTargets = 0;

    std::vector<double> timePointDistances;





	/**
		condition to remove a target
	*/
	struct removeCond {

		std::vector<int> removeIds;
		removeCond(std::vector<int> &removeIds) {
			this->removeIds = removeIds;
		}

		bool operator()(std::shared_ptr<Target> &target) {
			auto result = find(removeIds.begin(), removeIds.end(), target->getID());
			return result != removeIds.end();
		}

	};

	//========================= Functions ===================================================================

	/**
		functions for the threads
	*/
	void detectObjects();
	void sortObjects();
	void regognizeTargets();

	/**
		inital start of the threads
	*/
	void startThreads();

	/**
		read LED specific config from given file
	*/
	void readConfigFromFile(const std::string &configFilePath);

	/**
		calculate frequencies from the given parameters
	*/
	void calculateFrequencies(const int numIDs, const int disImages, const double frequency_low, const double frequency_high);
};


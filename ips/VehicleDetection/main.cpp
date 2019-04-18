#ifndef MAIN_CPP
#define MAIN_CPP

#include "default.h"
#include "CommunicationAPI.h"
#include "CommunicationAPI_LED.h"

#include "Evaluation.h"


struct Configuration {
	Mode mode;
	std::string configPath;
	std::string serial_no;

	int numIt;

	bool simulate;
	std::vector<std::string> filePaths;

	std::string evaluationPath;
	bool evaluate;
	bool printResults;
	bool printDetail;
	bool showImages;
	bool liveprint;
};


struct OverallEval {
	std::vector<double> positionErrors;
	std::vector<double> orientationErrors;
	std::vector<double> durations;
	std::vector<double> timepointsDistances;

};

OverallEval overallEval;


void prepare(Configuration &config) {

	std::ifstream file("Configuration/configMain.txt");

	if (!file.is_open()) {
		throw std::runtime_error(__FILE__ ": File could not be read! Config File invalid");
	}

	std::string line;
	while (getline(file, line)) {

		if (line.find("\"Mode\": ") == 0) {
			std::string value = line.substr(8);
			
			if (value == "LED") {
				config.mode = Mode::LED;
			}
			else if (value == "TAG") {
				config.mode = Mode::TAG;
			}
			else {
				throw std::runtime_error("unknown mode!");
			}
		}

		if (line.find("\"FilePath\":") == 0) {
			config.configPath = line.substr(12);
		}

		if (line.find("\"serial_no\":") == 0) {
			config.serial_no = line.substr(13);
		}

		if (line.find("\"numIterations\":") == 0) {
			std::string it = line.substr(17);
			config.numIt = std::stoi(it);
		}

		if (line.find("\"simulation\":") == 0) {
			std::string value = line.substr(14);
			config.simulate = (value == "True");
		}

		if (line.find("\"SimulationFilePaths\":") == 0) {
			std::vector<std::string> filePaths;
			size_t start = line.find_first_of("\"", 23);
			size_t end = line.find_first_of("\"", start + 1);
			while (start != std::string::npos) {
				filePaths.push_back(line.substr(start + 1, end - start - 1));
				start = line.find_first_of("\"", end + 1);
				end = line.find_first_of("\"", start + 1);
			}
			config.filePaths = filePaths;
		}

		if (line.find("\"Evaluation\":") == 0) {
			std::string value = line.substr(14);
			config.evaluate = (value == "True");
		}

		if (line.find("\"Print Results\":") == 0) {
			std::string value = line.substr(17);
			config.printResults = (value == "True");
		}

		if (line.find("\"Output Detail\":") == 0) {
			std::string value = line.substr(17);
			config.printDetail = (value == "True");
		}

		if (line.find("\"Show Images\":") == 0) {
			std::string value = line.substr(15);
			config.showImages = (value == "True");
		}

		if (line.find("\"LivePrint\":") == 0) {
			std::string value = line.substr(13);
			config.liveprint = (value == "True");
		}
	}

	config.evaluationPath = "Evaluation.txt";
}

void initialize(const Configuration &config,  std::shared_ptr<CommunicationAPI> &api) {

	api = std::make_shared<CommunicationAPI_LED>(config.serial_no, config.configPath);
}

//evaluation is easier with doubles than with long long
void run(const Configuration &config, std::shared_ptr<CommunicationAPI> const& api) {

	std::vector<Situation> situations;
	std::vector<double> durations;

    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t_start = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);



	for (int i = 0; i < config.numIt; i++) {

		Situation situation;

        clock_gettime(CLOCK_REALTIME, &t);
        uint64_t t1 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

		api->getCurrentSituation(situation);

        clock_gettime(CLOCK_REALTIME, &t);
        uint64_t t2 = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
		durations.push_back((t2 - t1)/1000.0);

		situations.push_back(situation);

		if (config.liveprint) {
			std::cout << "Duration Situation: " << durations[i] << " micro seconds" << std::endl;

			auto situation = situations[i];

			std::cout << "Situation at " << situation.timestamp << ":" << std::endl;
			for (auto it = situation.mapping.begin(); it != situation.mapping.end(); it++) {

				std::cout << "  id = " << it->first << std::endl;
				std::cout << "  position = " << it->second.position << std::endl;
				std::cout << "  orientation = " << it->second.orientation << std::endl << std::endl;
			}
			std::cout << std::endl;
		}

	}

    clock_gettime(CLOCK_REALTIME, &t);
    uint64_t t_end = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
	auto duration = (t_end - t_start) / 1000.0;

	api->cleanUp();

	std::vector<double> timePointDistances = api->getTimePointDistances();

	if (config.printDetail) {
		for (size_t i = 0; i < situations.size(); i++) {
			std::cout << "Duration Situation: " << durations[i] << " micro seconds" << std::endl;

			auto situation = situations[i];

			std::cout << "Situation at " << situation.timestamp << ":" << std::endl;
			for (auto it = situation.mapping.begin(); it != situation.mapping.end(); it++) {

				std::cout << "  id = " << it->first << std::endl;
				std::cout << "  position = " << it->second.position << std::endl;
				std::cout << "  orientation = " << it->second.orientation << std::endl << std::endl;
			}
			std::cout << std::endl;
		}
	}

	if (config.evaluate) {

		Evaluation eval(situations, durations);
		eval.setMaxErrors(0.01, 0.035);

		if (config.printResults) {
			eval.createFile(config.evaluationPath);
		}

		eval.calculateDuration(duration);
		eval.calculatePositionError();
		eval.calculateTimePointDistances(timePointDistances);


		std::vector<double> posErrors = eval.getErrorPos();
		std::vector<double> oriErrors = eval.getErrorOrientation();

		overallEval.durations.insert(overallEval.durations.end(), durations.begin(), durations.end());
		overallEval.timepointsDistances.insert(overallEval.timepointsDistances.end(), timePointDistances.begin(), timePointDistances.end());
		overallEval.positionErrors.insert(overallEval.positionErrors.end(), posErrors.begin(), posErrors.end());
		overallEval.orientationErrors.insert(overallEval.orientationErrors.end(), oriErrors.begin(),oriErrors.end());


		if (config.showImages) {
			//std::vector<int> idsToEvaluate = { 12, 30, 33, 57, 60, 63, 96, 99, 102, 144, 147, 150, 201, 204, 267 };
			std::vector<int> idsToEvaluate = { 60, 63 }; // , 99, 345

			for (int id : idsToEvaluate) {
				eval.illustrateCalcSituations(id);
				eval.illustrateRealSituations(500, id);
			}
		}

		eval.printFile();
	}
}


void simulate(Configuration &config, std::shared_ptr<CommunicationAPI> &api) {
	bool success;
	for (std::string filePath : config.filePaths) {
		success = api->setCamera(filePath);

		if (success) {
			int scenario = filePath.find("Scenario_");
			std::string end = filePath.substr(scenario + 8);
			config.evaluationPath = "Evaluation" + end;

			run(config, api);
		}
		
		initialize(config, api);
	}
}


int main() {

	Configuration config;
	prepare(config);

	std::shared_ptr<CommunicationAPI> api;
	initialize(config, api);

	if (config.simulate) {
		overallEval.durations = std::vector<double>();
		overallEval.timepointsDistances = std::vector<double>();
		overallEval.positionErrors = std::vector<double>();
		overallEval.orientationErrors = std::vector<double>();

		simulate(config, api);

		if (config.evaluate) {
			std::ofstream out("OverallEvaluation.txt");

			double min, max, mean, stdDev, median, upperQ, lowerQ, upperW, lowerW;
			if (!overallEval.durations.empty()) {
				ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(overallEval.durations, min, max, mean, stdDev);
				ImageProcessingHelper::getInstance().calculateBoxplot(overallEval.durations, median, upperQ, lowerQ, upperW, lowerW);
				out << "Durations: " << std::endl;
				out << "   Min    = " << min << std::endl;
				out << "   Max    = " << max << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Stddev = " << stdDev << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Q3     = " << upperQ << std::endl;
				out << "   Q1     = " << lowerQ << std::endl;
				out << "   upperW = " << upperW << std::endl;
				out << "   lowerW = " << lowerW << std::endl;
			}


			if (!overallEval.timepointsDistances.empty()) {
				ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(overallEval.timepointsDistances, min, max, mean, stdDev);
				ImageProcessingHelper::getInstance().calculateBoxplot(overallEval.timepointsDistances, median, upperQ, lowerQ, upperW, lowerW);
				out << std::endl << "Time Point Distances: " << std::endl;
				out << "   Min    = " << min << std::endl;
				out << "   Max    = " << max << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Stddev = " << stdDev << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Q3     = " << upperQ << std::endl;
				out << "   Q1     = " << lowerQ << std::endl;
				out << "   upperW = " << upperW << std::endl;
				out << "   lowerW = " << lowerW << std::endl;
			}


			if (!overallEval.positionErrors.empty()) {
				ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(overallEval.positionErrors, min, max, mean, stdDev);
				ImageProcessingHelper::getInstance().calculateBoxplot(overallEval.positionErrors, median, upperQ, lowerQ, upperW, lowerW);
				out << std::endl << "Position: " << std::endl;
				out << "   Min    = " << min << std::endl;
				out << "   Max    = " << max << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Stddev = " << stdDev << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Q3     = " << upperQ << std::endl;
				out << "   Q1     = " << lowerQ << std::endl;
				out << "   upperW = " << upperW << std::endl;
				out << "   lowerW = " << lowerW << std::endl;
			}

			if (!overallEval.orientationErrors.empty()) {
				ImageProcessingHelper::getInstance().calculateMinMaxMeanStdDev(overallEval.orientationErrors, min, max, mean, stdDev);
				ImageProcessingHelper::getInstance().calculateBoxplot(overallEval.orientationErrors, median, upperQ, lowerQ, upperW, lowerW);
				out << std::endl << "Orientation: " << std::endl;
				out << "   Min    = " << min << std::endl;
				out << "   Max    = " << max << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Stddev = " << stdDev << std::endl;
				out << "   Mean   = " << mean << std::endl;
				out << "   Q3     = " << upperQ << std::endl;
				out << "   Q1     = " << lowerQ << std::endl;
				out << "   upperW = " << upperW << std::endl;
				out << "   lowerW = " << lowerW << std::endl;
			}

			out.close();
		}
	}
	else {
		bool success = api->setCamera();
		if (success) {
			run(config, api);
		}
	}
		

	cv::Mat image = cv::imread("Situations/Vehicle1_On.png", cv::IMREAD_GRAYSCALE);
	cv::namedWindow("Display window", cv::WINDOW_NORMAL);
	cv::imshow("Display window", image);
	cv::resizeWindow("Display window", 500, 500);

	cv::waitKey(500000);

	return 0;
}


int main___() {
	std::ofstream out = std::ofstream("help.txt");

	out << "\"../IPS_Simulation/Scenarios/Scenario_1.txt\"";
	for (int i = 1; i < 33; i++) {
		out << " \"../IPS_Simulation/Scenarios/Scenario_2_" << i << ".txt\"";
	}

	out << " \"../IPS_Simulation/Scenarios/Scenario_3_1.txt\"";
	out << " \"../IPS_Simulation/Scenarios/Scenario_3_2.txt\"";

	out << " \"../IPS_Simulation/Scenarios/Scenario_4_1.txt\"";
	out << " \"../IPS_Simulation/Scenarios/Scenario_4_2.txt\"";


	for (int i = 1; i < 41; i++) {
		out << " \"../IPS_Simulation/Scenarios/Scenario_5_" << i << ".txt\"";
	}

	out.close();
	return 0;
}

#endif
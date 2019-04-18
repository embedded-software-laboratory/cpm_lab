#ifndef MAIN_LIVE_CPP
#define MAIN_LIVE_CPP

#include <csignal>

#include "default.h"
#include "CommunicationAPI.h"
#include "CommunicationAPI_LED.h"
#include "VehicleObservation.hpp"
#include <dds/pub/ddspub.hpp>


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


bool continueLoop = true;


// TODO de-duplicate this wrt main.cpp
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

void handleStrgC(int s) {
    if(s == 2) {
        //strg-c
        continueLoop = false;
    }
}


int main() {


    std::signal(SIGINT, handleStrgC);

    Configuration config;
    prepare(config);

    std::shared_ptr<CommunicationAPI> api;
    initialize(config, api);

    config.simulate = false;

    ThreadSafeQueue<Situation> situationQueue;


    //std::cout << config.filePaths.front() << std::endl;
    //api->setCamera(config.filePaths.front());
    api->setCamera();


    /// DDS Setup
    dds::domain::DomainParticipant dds_participant(0);
    dds::topic::Topic<VehicleObservation> topic_vehicleObservation(dds_participant, "vehicleObservation");
    dds::pub::DataWriter<VehicleObservation> writer_vehicleObservation(dds::pub::Publisher(dds_participant), topic_vehicleObservation);


    while(continueLoop) {
        Situation situation;
      
        api->getCurrentSituation(situation);

       // std::cout << "Situation at " << situation.timestamp << ":" << std::endl;

        for (auto it = situation.mapping.begin(); it != situation.mapping.end(); it++) {
          /*  std::cout << "  id = " << it->first << std::endl;
            std::cout << "  position = " << it->second.position << std::endl;
            std::cout << "  orientation = " << it->second.orientation << std::endl << std::endl;*/


            /// DDS broadcast

            // round the timestamp to the previous 20ms tick
            // this is quick & dirty, and will cause a slight error in positioning
            // TODO interpolate with the previous detection to get a better positioning for the tick time

            int64_t tick = situation.timestamp / 20000000;
            int64_t stamp = tick * 20000000;

            if(it->first > 0) // ID=0 is invalid, ignore
            {
                VehicleObservation vehicleObservation;
                vehicleObservation.vehicle_id(it->first);
                vehicleObservation.pose().x(it->second.position.x);
                vehicleObservation.pose().y(it->second.position.y);
                vehicleObservation.pose().yaw(it->second.orientation);
                vehicleObservation.header().create_stamp().nanoseconds(stamp);
                vehicleObservation.header().valid_after_stamp().nanoseconds(stamp);

                writer_vehicleObservation.write(vehicleObservation);
            }
        }
        
      //  std::cout << std::endl;
    }

    api->cleanUp();
    writer_vehicleObservation.close();
    std::cout << "end clean up" << std::endl;

    return 0;
}

#endif
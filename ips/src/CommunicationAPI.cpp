#include "CommunicationAPI.h"
#include <fstream>

CommunicationAPI::CommunicationAPI(const std::string &serial_no, const std::string &configFilePath)
{
	this->serial_no = serial_no;
	setCameraParameters(serial_no);
	readStandardFromFile(configFilePath);
}


CommunicationAPI::~CommunicationAPI()
{
}


void CommunicationAPI::addVehicle(const int id, const IdentificationCharacteristic &idChar) {

	if (vehicles.count(idChar) > 0) {
		throw std::runtime_error("API: Id Characteristic already in map");
	}

	vehicles[idChar] = id;
}

void CommunicationAPI::cleanUp() {

}

bool CommunicationAPI::setCamera(const std::string &filename) {
	return true;
}

void CommunicationAPI::setCameraParameters(const std::string &serial_no) {
	std::string extrinsic_parameters_path = "cfg/cameras/" + serial_no + "/extrinsic_parameters.yaml";
	std::string intrinsic_parameters_path = "cfg/cameras/" + serial_no + "/intrinsic_parameters.yaml";

	std::shared_ptr<CameraParameter> params = std::make_shared<CameraParameter>();

	params->setExtrinsicParametersFromYAML(extrinsic_parameters_path);
	params->setIntrinsicParametersFromYAML(intrinsic_parameters_path);

	params->initializeMatrices();

	GlobalDataHelper::getInstance().setCameraParameters(params);
}

void CommunicationAPI::readStandardFromFile(const std::string &configFilePath) {

	std::ifstream file(configFilePath);

	if (!file.is_open()) {
		throw std::runtime_error(__FILE__ ": File could not be read! Config File invalid");
	}

	bool referenceLine = false;
	bool vehicle_height = false;
	bool vehicle_roof = false;
	bool exposure = false;

	double height, roof_height;
	int exposure_time;

	std::string line;
	while (getline(file, line)) {
		if (line.find("\"reference Line\": ") == 0) {

			std::string point1, point2;
			size_t pos;
			FileReaderHelper::getInstance().findNextPoint(line, 0, point1, pos);
			FileReaderHelper::getInstance().findNextPoint(line, pos, point2, pos);

			WorldPoint startLine = FileReaderHelper::getInstance().getWorldPointFromString(point1);
			WorldPoint endLine = FileReaderHelper::getInstance().getWorldPointFromString(point2);

			GlobalDataHelper::getInstance().setSlopeOfReferenceLine(startLine, endLine);
			referenceLine = true;
		}

		if (line.find("\"HEIGHT_VEHICLE\": ") == 0) {
			height = FileReaderHelper::getInstance().getDoubleFromString(line, 18);
			vehicle_height = true;
		}

		if (line.find("\"HEIGHT_VEHICLE_ROOF\": ") == 0) {
			roof_height = FileReaderHelper::getInstance().getDoubleFromString(line, 24);
			vehicle_roof = true;
		}

		if (line.find("\"Exposure Time\": ") == 0) {
			exposure_time = FileReaderHelper::getInstance().getIntFromString(line, 17);
			exposure = true;
		}
	}

	file.close();

	assert(vehicle_height && vehicle_roof && referenceLine && exposure);

	GlobalDataHelper::getInstance().setGlobalConst(height, roof_height, exposure_time);
}
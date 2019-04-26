#include "FileReaderHelper.h"

bool FileReaderHelper::findNextCharPair(const std::string &s, const std::string &c_1, const std::string &c_2, size_t pos, std::string &result, size_t &end_result_pos) {
	size_t start = s.find_first_of(c_1, pos);
	size_t end = s.find_first_of(c_2, start + 1);
	if (start == std::string::npos) {
		return false;
	}
	result = s.substr(start, end - start);
	end_result_pos = end;
	return true;
}

bool FileReaderHelper::findNextPoint(const std::string &s, size_t pos, std::string &point, size_t &end_point_pos) {
	return findNextCharPair(s, "[", "]", pos, point, end_point_pos);
}

bool FileReaderHelper::findNextMapping(const std::string &s, size_t pos, std::string &map, size_t &end_map_pos) {
	return findNextCharPair(s, "{", "}", pos, map, end_map_pos);
}

WorldPoint FileReaderHelper::getWorldPointFromString(const std::string &point) {
	size_t y_pos = point.find_first_of(",");
	size_t z_pos = point.find_first_of(",", y_pos + 1);

	std::string x_value = point.substr(1, y_pos - 1);
	std::string y_value = point.substr(y_pos + 2, z_pos - (y_pos + 2));
	std::string z_value = point.substr(z_pos + 2, point.length() - (z_pos + 2));

	return WorldPoint(std::stod(x_value), std::stod(y_value), std::stod(z_value));
}

WorldPoint FileReaderHelper::getWorldPointWithConstantZFromString(const std::string &point) {
	size_t y_pos = point.find_first_of(",");
	size_t z_pos = point.find_first_of(",", y_pos + 1);

	std::string x_value = point.substr(1, y_pos - 1);
	std::string y_value = point.substr(y_pos + 2, z_pos - (y_pos + 2));
	std::string z_value = point.substr(z_pos + 2, point.length() - (z_pos + 2));

	double z = 0.0;
	if (z_value == "HEIGHT_VEHICLE") {
		z = GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE;
	}
	else if (z_value == "HEIGHT_VEHICLE_ROOF") {
		z = GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE_ROOF;
	}
	else if (z_value.find_first_not_of("0123456789.") == std::string::npos) {
		//only numbers
		z = std::stod(z_value);
	}

	return WorldPoint(std::stod(x_value), std::stod(y_value), z);

}

double FileReaderHelper::getDoubleFromString(const std::string &s, size_t pos) {
	std::string value = s.substr(pos, s.length() - pos);
	return std::stod(value);
}

double FileReaderHelper::getDoubleFromString(const std::string &s, size_t pos_1, size_t pos_2) {
	std::string value = s.substr(pos_1, pos_2 - pos_1);
	return std::stod(value);
}

int FileReaderHelper::getIntFromString(const std::string &s, size_t pos) {
	std::string value = s.substr(pos, s.length() - pos);
	return std::stoi(value);
}
int FileReaderHelper::getIntFromString(const std::string &s, size_t pos_1, size_t pos_2) {
	std::string value = s.substr(pos_1, pos_2 - pos_1);
	return std::stoi(value);
}

void FileReaderHelper::readGeometrieFromString(const std::string &s, Geometrie &geometrie) {
	
	size_t idPts = s.find_first_of(";");

	std::vector<WorldPoint> positionPoints;
	std::vector<WorldPoint> identificationPoints;

	std::string point;
	size_t pos = 0;
	size_t end_point_pos;

	while (findNextPoint(s, pos, point, end_point_pos)) {

		if (end_point_pos < idPts) {
			positionPoints.push_back(getWorldPointWithConstantZFromString(point));
		}
		else {
			identificationPoints.push_back(getWorldPointWithConstantZFromString(point));
		}
		pos = end_point_pos;
	}

	geometrie.positionPoints = positionPoints;
	geometrie.identificationPoints = identificationPoints;
}


Interval FileReaderHelper::readIntervalFromString(const std::string &point) {
	size_t pos = point.find_first_of(",");

	std::string first_value = point.substr(1, pos - 1);
	std::string second_value = point.substr(pos + 2, point.length() - (pos + 2));

	return Interval(std::stod(first_value), std::stod(second_value));
}
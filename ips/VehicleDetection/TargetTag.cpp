#include "TargetTag.h"


TargetTag::TargetTag(const Tag &code)
{
	this->code = code;
	calculateIdentification();
	calculatePositionAndOrientation();
}


TargetTag::~TargetTag()
{
}


void TargetTag::calculateIdentification() {
	id = std::stoi(code.data);
}

void TargetTag::calculatePositionAndOrientation() {

	location.orientation = ImageProcessingHelper::getInstance().calculateOrientation(code.location[2], code.location[1], code.location[3], GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE_ROOF);

	WorldPoint tagPosition = GlobalDataHelper::getInstance().getCameraParameters()->reprojectPoint(code.location[2], GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE_ROOF);
	WorldPoint offset_vehicle = GlobalDataHelper::getInstance().getGeometrie().positionPoints.front();

	double x_offset, y_offset;

	if (location.orientation == 0) {
		x_offset = - offset_vehicle.y;
		y_offset = std::abs(offset_vehicle.x);
	}
	else if (location.orientation == M_PI / 2) {
		x_offset = - std::abs(offset_vehicle.x);
		y_offset = - offset_vehicle.y;
	}
	else if (location.orientation == M_PI || location.orientation == -M_PI) {
		x_offset = offset_vehicle.y;
		y_offset = - std::abs(offset_vehicle.x);
	}
	else if (location.orientation == -M_PI / 2) {
		x_offset = std::abs(offset_vehicle.x);
		y_offset = offset_vehicle.y;
	} 
	else if (0 < location.orientation && location.orientation < M_PI/2) {
		double a = sin(location.orientation) * offset_vehicle.y;
		double e = sin(M_PI / 2 - location.orientation) * std::abs(offset_vehicle.x);
		double c = offset_vehicle.y * sqrt(1 - pow(sin(location.orientation), 2));
		double f = std::abs(offset_vehicle.x) * sqrt(1 - pow(sin(M_PI/2 - location.orientation), 2));
		x_offset = -(c + f);
		y_offset = e - a;
	}
	else if (M_PI/2 < location.orientation && location.orientation < M_PI) {
		double a = sin(M_PI - location.orientation) * offset_vehicle.y;
		double e = sin(location.orientation - M_PI/2) * std::abs(offset_vehicle.x);
		double c = offset_vehicle.y * sqrt(1 - pow(sin(M_PI - location.orientation), 2));
		double f = std::abs(offset_vehicle.x) * sqrt(1 - pow(sin(location.orientation - M_PI / 2), 2));
		x_offset = c + f;
		y_offset = -(e + a);
	}
	else if (0 > location.orientation && location.orientation > - M_PI / 2) {
		double a = sin(location.orientation) * offset_vehicle.y;
		double e = sin(M_PI / 2 + location.orientation) * std::abs(offset_vehicle.x);
		double c = offset_vehicle.y * sqrt(1 - pow(sin(location.orientation), 2));
		double f = std::abs(offset_vehicle.x) * sqrt(1 - pow(sin(M_PI / 2 + location.orientation), 2));
		x_offset = - (c + f);
		y_offset = e - a;
	}
	else if (-M_PI/2 > location.orientation && location.orientation > - M_PI) {
		double a = sin(M_PI - location.orientation) * offset_vehicle.y;
		double e = sin(location.orientation - M_PI / 2) * std::abs(offset_vehicle.x);
		double c = offset_vehicle.y * sqrt(1 - pow(sin(M_PI - location.orientation), 2));
		double f = std::abs(offset_vehicle.x) * sqrt(1 - pow(sin(location.orientation - M_PI / 2), 2));
		x_offset = c + f;
		y_offset = -(e + a);
	}

	location.position = WorldPoint(tagPosition.x + x_offset, tagPosition.y + y_offset, tagPosition.z);
}
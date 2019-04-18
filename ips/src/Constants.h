/**
	global constants describing the
	height of the vehicle and the 
	exposure time of the camera
*/
struct GlobalConstants {

	double HEIGHT_VEHICLE;
	double HEIGHT_VEHICLE_ROOF;

	int EXPOSURE_TIME;
};

/**
	Constants for the LED approach:
	frequency of the camera
	offset angle of the right side of the vehicle
	offset angle of the left side of the vehicle

	tolerances for the shortest distance, the longest distance in geometrie
	and tolerance whether a point is in range or not
*/
struct LEDConstants {
	double CAMERA_FREQUENCY;

	double OFFSET_RIGHT;
	double OFFSET_LEFT;

	double TOLERANCE_SHORTEST_LOW = 4;
	double TOLERANCE_SHORTEST_HIGH = 4;
	double TOLERANCE_LONGEST = 1.1;
	double TOLERANCE_RANGE = 10;

	double TOLERANCE_GEOMETRY_DISTANCES = 400;
};



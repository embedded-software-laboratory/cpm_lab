#include "CameraFileWrapper.h"
#include <fstream>

//the cantor pairing always returns an integer value only stored as double - no lost of data
CameraFileWrapper::CameraFileWrapper(const std::string &serial_no, const std::string &filename)
	: Camera(serial_no)
{
	std::map<int, int> id_simulation_to_id;

	int id_simulation = 0;

	for (int i = 2; i < 15; i = i + 3) {
		for (int j = 2; j <15; j = j + 3)  {
			//cantor pairing function - always returns int
			int id = 0.5 * (i + j) * (i + j + 1) + j;
			id_simulation_to_id[id_simulation] = id;
			id_simulation++;
		}
	}


	//read simulation
	std::vector<Situation> situations;

	std::ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "File \"" << filename << "\" could not be read! No images" << std::endl;
		GlobalDataHelper::getInstance().cancelProgram();
		return;
	}

	std::string line;
	Time_Stamp timestamp;
	while (getline(file, line)) {
		std::string beginning = line.substr(0, 13);
		if (beginning == "\"timestamp\": ") {
			std::string stamp = line.substr(13, line.length() - 13);
			timestamp = std::stoll(stamp);
			timestamps.push_back(timestamp);
			continue;
		}

		if (beginning == "\"light_points") {
			std::vector<ImagePoint> points;
			size_t start = line.find_first_of("{");
			size_t end = line.find_first_of("}");
			while (start != std::string::npos) {
				std::string point = line.substr(start, end - start);
				size_t y_pos = point.find("y");
				std::string x_value = point.substr(5, y_pos - 8);
				std::string y_value = point.substr(y_pos + 3, point.length() - y_pos - 3);
				WorldPoint worldPoint(std::stod(x_value), std::stod(y_value), GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);
				points.push_back(GlobalDataHelper::getInstance().getCameraParameters()->projectPoint(worldPoint));
				start = line.find_first_of("{", end);
				end = line.find_first_of("}", start);
			}
			pointsPerImageFromFile.push_back(points);
		}

		if (beginning == "\"real_points\"") {
			Situation situation;
			situation.timestamp = timestamp;

			size_t start = line.find_first_of("{");
			size_t end = line.find_first_of("}");
			while (start != std::string::npos) {
			std::string vehicle = line.substr(start, end - start);

			size_t id_pos = vehicle.find_first_of(":", 6);
			size_t x_pos = vehicle.find("x");
			size_t y_pos = vehicle.find("y");
			size_t alpha_pos = vehicle.find("orientation");

			std::string id_value = vehicle.substr(6, id_pos - 6);
			std::string x_value = vehicle.substr(x_pos + 3, y_pos - (3 + 3 + x_pos));
			std::string y_value = vehicle.substr(y_pos + 3, alpha_pos - (y_pos + 3 + 3));
			std::string orientation_value = vehicle.substr(alpha_pos + 13, vehicle.length() - (alpha_pos + 13));

			Location location;

			location.position = WorldPoint(std::stod(x_value), std::stod(y_value), GlobalDataHelper::getInstance().getGlobalConst().HEIGHT_VEHICLE);
			location.orientation = std::stod(orientation_value);

			if (location.orientation > M_PI) {
				while (location.orientation > M_PI) {
					location.orientation -= 2 * M_PI;
				}
			}

			int id = id_simulation_to_id[std::stoi(id_value)];

			situation.mapping[id] = location;

			start = line.find_first_of("{", end);
			end = line.find_first_of("}", start);
			}

			situations.push_back(situation);
		}
	}

	file.close();

	GlobalDataHelper::getInstance().setRealSituations(situations);

	currentImage = 0;

	grabbingThread = std::thread([this] { this->grabbingImages(); });
}


CameraFileWrapper::~CameraFileWrapper()
{
}

void CameraFileWrapper::getPointsInImages(std::vector<ImagePoint> &pointsPerImage, Time_Stamp &timestamp) {
	pointsPerImage = pointsPerImageFromFile[currentImage];
	currentImage ++;
	if (currentImage >= pointsPerImageFromFile.size()) {
		currentImage = pointsPerImageFromFile.size() - 1;
	}
}


void CameraFileWrapper::grabbingImages() {
	while (continue_grabbing) {
		cv::waitKey(16);
		std::vector<ImagePoint> pointPerImage;
		Time_Stamp timestamp;
		cv::Mat img = cv::Mat::zeros(2048, 2048, CV_8UC1);
		getPointsInImages(pointPerImage, timestamp);
		drawPoints(img, pointPerImage);

        struct timespec t;
        clock_gettime(CLOCK_REALTIME, &t);
        timestamp = uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);

		images.push(TimedImage(id, img, timestamp));
		id++;
		img.release();
	}
}

bool CameraFileWrapper::grabImage(std::shared_ptr<TimedImage> &image) {
	image = std::make_shared<TimedImage>(images.pop());
	return true;
}

void CameraFileWrapper::close() {
	continue_grabbing = false;
	grabbingThread.join();
}


void CameraFileWrapper::drawPoints(const cv::Mat &image, const std::vector<ImagePoint> &points) {
	for (ImagePoint point : points) {
		cv::circle(image, point, 2, cv::Scalar(255, 255, 255), cv::FILLED);
	}
}

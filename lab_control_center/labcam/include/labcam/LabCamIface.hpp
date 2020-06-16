#pragma once
#include <string>
#include <thread>

class LabCam;

class LabCamIface {
	private:
		LabCam* impl_;

	public:
		LabCamIface();
		~LabCamIface() = default;

		void startRecording(std::string path, std::string file_name);
		void stopRecording();
};
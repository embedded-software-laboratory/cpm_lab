#pragma once
#include <string>
#include <thread>

class LabCam;

/**
 * \brief This module provides standardized access to the LabCam.
 * \ingroup lcc_labcam
 */
class LabCamIface {
	private:
		//! Contains the actual LabCam object.
		LabCam* impl_;

	public:
		/**
		 * \brief Default Constructor
		 */
		LabCamIface();
		~LabCamIface() = default;

		/**
		 * \brief Starts recording of LabCam.
		 * \param path Folder in which the resulting video will be saved.
		 * \param file_name Filename of the recording.
		 */
		void startRecording(std::string path, std::string file_name);

		/**
		 * \brief Stops current recording.
		 */
		void stopRecording();
};
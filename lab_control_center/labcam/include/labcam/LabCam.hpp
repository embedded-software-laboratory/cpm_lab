#pragma once
#include <string>
#include <thread>
#include "CInstantCameraAppSrc.h"
#include <gst/gst.h>


/**
 * \brief Contains the actual implementation of the LabCam.
 * \ingroup lcc_labcam
 */
class LabCam {
	public:
		/**
		 * \brief Type for specifying the current status of the labcam.
		 */
		typedef enum {
			CAM_STATUS_RECORDING,
			CAM_STATUS_STOPPED,
			CAM_STATUS_ERROR,
		} LAB_CAM_STATUS_T;
	private:
		//! TODO
		std::thread cam_thread_;
		//! TODO
        LAB_CAM_STATUS_T cam_status_;

		//! TODO
        bool thread_running_;

		//! TODO
		GMainLoop *loop = nullptr;
		//! TODO
		GstBus *bus = nullptr;
		//! TODO
		guint bus_watch_id;
		//! TODO
		GstElement *pipeline = nullptr;

		//! Filename of the recording
		std::string file_name_;
		//! Folder in which the resulting video will be saved
		std::string path_;

        //! A pipeline needs a source element. The InstantCameraForAppSrc will create, configure, and provide an AppSrc which fits the camera.
        GstElement *source = nullptr;
        //! Create the other needed gstreamer pipeline elements
        GstElement *convert = nullptr;
		//! TODO
        GstElement *sink = nullptr;
		//! TODO
        GstElement *x264enc = nullptr;

		/**
		 * \brief TODO
		 * \param data TODO
		 */
		static void jumppad(gpointer data);

		/**
		 * \brief Starts recording
		 */
		bool startRecordingImpl();

	public:
		LabCam() = default;
		~LabCam() = default;

		/**
		 * \brief Wrapper to specify the output folder/file and to start the recording of the LabCam.
		 * \param path Folder in which the resulting video will be saved
		 * \param file_name Filename of the recording
		 */
		void startRecording(std::string path, std::string file_name);

		/**
		 * \brief Stops current recording
		 */
		void stopRecording();
};
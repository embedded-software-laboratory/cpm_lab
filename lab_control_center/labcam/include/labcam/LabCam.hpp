#pragma once
#include <string>
#include <thread>
#include "CInstantCameraAppSrc.h"
#include <gst/gst.h>

class LabCam {
	public:
		typedef enum {
			CAM_STATUS_RECORDING,
			CAM_STATUS_STOPPED,
			CAM_STATUS_ERROR,
		} LAB_CAM_STATUS_T;
	private:
		std::thread cam_thread_;
        LAB_CAM_STATUS_T cam_status_;

        bool thread_running_;

		GMainLoop *loop = nullptr;
		GstBus *bus = nullptr;
		guint bus_watch_id;
		GstElement *pipeline = nullptr;

		std::string file_name_;
		std::string path_;

        // A pipeline needs a source element. The InstantCameraForAppSrc will create, configure, and provide an AppSrc which fits the camera.
        GstElement *source = nullptr;
        // Create the other needed gstreamer pipeline elements
        GstElement *convert = nullptr;
        GstElement *sink = nullptr;
        GstElement *x264enc = nullptr;


		static void jumppad(gpointer data);
		bool startRecordingImpl();

	public:
		LabCam() = default;
		~LabCam() = default;

		void startRecording(std::string path, std::string file_name);
		void stopRecording();
};
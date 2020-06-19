// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
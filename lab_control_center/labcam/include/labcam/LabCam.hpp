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


/**
 * \brief TODO
 * \ingroup lcc_labcam
 */
class LabCam {
	public:
		/**
		 * \brief TODO
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

		//! TODO
		std::string file_name_;
		//! TODO
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
		 * \brief TODO
		 */
		bool startRecordingImpl();

	public:
		LabCam() = default;
		~LabCam() = default;

		/**
		 * \brief TODO
		 * \param path TODO
		 * \param file_name TODO
		 */
		void startRecording(std::string path, std::string file_name);

		/**
		 * \brief TODO
		 */
		void stopRecording();
};
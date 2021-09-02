#include "labcam/LabCamIface.hpp"
#include "labcam/LabCam.hpp"

/**
 * \file LabCamIface.cpp
 * \ingroup lcc_labcam
 */

LabCamIface::LabCamIface(){
	impl_ = new LabCam();
}

void LabCamIface::startRecording(std::string path, std::string file_name){
	impl_->startRecording(path, file_name);
}

void LabCamIface::stopRecording(){
	impl_->stopRecording();
}
#pragma once

#include "default.h"
#include "FileReaderHelper.h"

class CommunicationAPI
{
public:
	CommunicationAPI(const std::string &serial_no, const std::string &configFilePath);
	~CommunicationAPI();


	/**
		get the current situation that means for each vehicle 
		added to the database its position and orientation
	*/
	virtual void getCurrentSituation(Situation &situation) = 0;

	/**
		set camera depending on mode
		if camera set to cameraFileWrapper provide filename
		returns whether it was successful
		if false is returned no camera was found
	*/
	virtual bool setCamera() = 0;
	virtual bool setCamera(const std::string &filename);

	/**
		clean up before deleting communication API
	*/
	virtual void cleanUp();

	/**
		add a vehicle to the internal database
		for each vehicle save the id and its characteristic
	*/
	void addVehicle(const int id, const IdentificationCharacteristic &idChar);

	virtual std::vector<double> getTimePointDistances() {
		return std::vector<double>();
	}


protected:

	/**
		serial number of the camera
	*/
	std::string serial_no;


	/**
		stores all vehicles that are known
	*/
	std::map<IdentificationCharacteristic, int> vehicles;

private:
	/**
		read parameters from file that are relevant to the complete
		system
	*/
	void readStandardFromFile(const std::string &configFilePath);

	/**
		set intrinic and extrinsic camera parameters for the specified camera
	*/
	void setCameraParameters(const std::string &serial_no);

};


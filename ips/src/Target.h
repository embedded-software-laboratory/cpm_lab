#pragma once

#include "default.h"
#include "ImageProcessingHelper.h"


class Target
{
public:

	virtual ~Target() {};


	/**
		get id of the target
	*/
	int getID() {
		return id;
	}

	/**
		get current location of the target
	*/
	void getLocation(Location &location) {
		location = this->location;
	}

	/**
		get current center of the target
	*/
	ImagePoint getCenter() {
		return center;
	}

	/**
		timestamp of the last update of this target
	*/
	Time_Stamp getTimeStamp() {
		return timestamp;
	}

protected:

	/**
		id of the target
	*/
	int id = 0;

	/**
		current location of the target
	*/
	Location location;

	/**
		timestamp of the update of the Target
	*/
	Time_Stamp timestamp;

	/**
		current center of the target
	*/
	ImagePoint center;

};

struct Targets {
	/**
		list of targets
	*/
	std::list<std::shared_ptr<Target>> elements;
	/**
		time to which this situation has occured
	*/
	Time_Stamp timestamp;
};


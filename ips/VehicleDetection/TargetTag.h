#pragma once
#include "Target.h"

class TargetTag :
	public Target
{
public:
	TargetTag(const Tag &code);
	~TargetTag();

private:

	/*
		tag which was found in the image
	*/
	Tag code;

	/*
		calculate identification from the tag
	*/
	void calculateIdentification();

	/*
		calculate the position and orientation from the tag
	*/
	void calculatePositionAndOrientation();
};


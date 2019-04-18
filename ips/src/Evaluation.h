#pragma once
#include "default.h"
#include "GlobalDataHelper.h"
#include <fstream>

class Evaluation
{
public:
	Evaluation(const std::vector<Situation> &situations, const std::vector<double> &durations);
	~Evaluation();

	/**
		create file where evaluation should be printed in
	*/
	void createFile(const std::string &filePath);

	/**
		close file of evaluation
	*/
	void printFile();

	/**
		set max errors for positioning and orientation
		to evaluate how often they are exceeded
	*/
	void setMaxErrors(const double maxErrorPos, const double maxErrorOrientation);

	/**
		calculate position error with the real situations and the calculated situations
		calculate min max mean and std dev
	*/
	void calculatePositionError();

	/**
		calculate min max mean and std dev and print duration sum
	*/
	void calculateDuration(const long long duration_sum);

	/**
		calculate min max mean and std dev
	*/
	void calculateTimePointDistances(std::vector<double> &timePointDistances);

	/**
		for eachs situation output timestamp, duration, id, position and orientation
	*/
	void printEachSituation();

	/**
		returns all errors of positioning
		-> for overall evaluation
	*/
	std::vector<double> getErrorPos();

	/**
		returns all errors of orientation
		-> for overall evaluation
	*/
	std::vector<double> getErrorOrientation();
	
	/**
		illustrate real situations for vehicle with given id
		only draw the first number situations
	*/
	void illustrateRealSituations(const size_t number, const int id);

	/**
		illustrate the calculated situations for the vehicle
		with the given id
	*/
	void illustrateCalcSituations(const int id);


private:
	/**
		real situations given by the simulation
	*/
	std::vector<Situation> realSituations;

	/**
		calculated situations while running the programm
	*/
	std::vector<Situation> calcSituations;
	
	/**
		durations for each situation
	*/
	std::vector<double> durations;

	/**
		maximal allowed error for positioning 
	*/
	double maxErrorPos = DBL_MAX;

	/**
		maximal allowed eroor for orientation
	*/
	double maxErrorOri = DBL_MAX;

	/**
		errors of positioning for each situation
	*/
	std::vector<double> error_pos;

	/**
		errors of orientation for each situation
	*/
	std::vector<double> error_orientation;

	/**
		start point from which the current real situation is searched in the vector
	*/
	size_t startPoint = 0;

	/**
		output file of the evaluation
	*/
	std::ofstream out;

	/**
		whether the evaluation should be outputted in the console
		or into a file
	*/
	bool writeToFile = false;

	/**
		draw situations of a specified id
		where the position is drawn as a point and the orientation as an arrow
	*/
	void illustrate(const std::vector<Situation> &situations, const int id, const bool saveToFile);

	/**
		draw into the given image a location: the position as point and 
		the orientation as arrow in the specified direction
	*/
	void drawLocation(const cv::Mat &image, const Location &location);

	/**
		draw into the given image a location: the position as point and 
		the orientation as arrow in the specified direction
		and specify the colors for position and orientation
	*/
	void drawLocation(const cv::Mat &image, const Location &location, const cv::Scalar &color1, const cv::Scalar &color2);

	/**
		find the real location at a specific timepoint for a given vehicle
	*/
	bool getRealLocationAtTime(const Time_Stamp &timestamp, const int vehicle, Location &location);

	/**
		calculate min max mean and std dev of a vector of doubles
	*/
	void calculateMinMaxMean(const std::vector<double> &values);
};




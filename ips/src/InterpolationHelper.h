#pragma once

#include "default.h"
#include "../ALGLIB/src/interpolation.h"
#include "../ALGLIB/src/stdafx.h"

class InterpolationHelper
{
public:

	/**
		provide time points and the corresponding positions at these points
		such that the i-th time point belongs to the i-th value
		additionally provide the degree of interpolation (1-linear, 2- quadratic, ..)
	*/
	InterpolationHelper(
		const std::vector<Time_Stamp> &timepoints, 
		const std::vector<WorldPoint> &values,
		const int degree);
	~InterpolationHelper();

	/**
		predict WorldPoint at time point timestamp
		according to the interpolation of the provided values
	*/
	WorldPoint getValueAt(const Time_Stamp &timestamp);

	/**
		get slope of polynomial at timestamp seperate for x and y
	*/
	std::pair<double, double> getSlopeAt(const Time_Stamp &timestamp);
	
private:
	/**
		relative origin of the calculations
	*/
	Time_Stamp startPoint;

	/**
		save height of the provided worldpoints for the output
	*/
	double height;

	/**
		degree of interpolation
		1 - linear
		2 - quadratic and so on
	*/
	int degree;

	/**
		list of timepoints relative to the origin
	*/
	alglib::real_1d_array timepoints;

	/**
		provided x values corresponding to the timepoints
	*/
	alglib::real_1d_array x_values;

	/**
		provided y values corresponding to the timepoints
	*/
	alglib::real_1d_array y_values;

	/**
		interpolate x data and predict x value at time t relative to origin
	*/
	double x_valueAt(const double t);

	/**
		interpolate y data and predict y value at time t relative to origin
	*/
	double y_valueAt(const double t);

	/**
		interpolate x data and predict x derivative at time t relative to origin
	*/
	double x_derivativeAt(const double t);

	/**
		interpolate y data and predict y derivative at time t relative to origin
	*/
	double y_derivativeAt(const double t);

	/**
		interpolate provided values with timepoints at time t
	*/
	double interpolateValue(const alglib::real_1d_array &values, const double t);

	/**
		interpolate derivate of values at time t
	*/
	double interpolateDerivative(const alglib::real_1d_array &values, const double t);

};


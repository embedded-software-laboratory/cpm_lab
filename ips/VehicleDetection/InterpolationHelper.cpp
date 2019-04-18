#include "InterpolationHelper.h"


InterpolationHelper::InterpolationHelper(
	const std::vector<Time_Stamp> &timepoints,
	const std::vector<WorldPoint> &values,
	const int degree)
{
	startPoint = timepoints.front();

	std::vector<double> help_t, help_x, help_y;
	for (Time_Stamp timepoint : timepoints) {
		//time points in ms
		double t = (timepoint - startPoint) / 1000.0;
		help_t.push_back(t);
	}

	for (WorldPoint value : values) {
		help_x.push_back(value.x);
		help_y.push_back(value.y);
	}

	this->timepoints.setcontent(help_t.size(), help_t.data());

	x_values.setcontent(help_x.size(), help_x.data());
	y_values.setcontent(help_y.size(), help_y.data());

	height = values.front().z;
	this->degree = degree;
}


InterpolationHelper::~InterpolationHelper()
{
}


WorldPoint InterpolationHelper::getValueAt(const Time_Stamp &timestamp) {
	double t = (timestamp - startPoint) / 1000.0;

	double x = x_valueAt(t);
	double y = y_valueAt(t);

	return WorldPoint(x, y, height);
}

std::pair<double, double> InterpolationHelper::getSlopeAt(const Time_Stamp &timestamp) {
	double t = (timestamp - startPoint) / 1000.0;

	double x = x_derivativeAt(t);
	double y = y_derivativeAt(t);

	return std::pair<double, double>(x,y);
}

double InterpolationHelper::x_valueAt(const double t) {
	return interpolateValue(x_values, t);
}


double InterpolationHelper::y_valueAt(const double t) {
	return interpolateValue(y_values, t);
}

double InterpolationHelper::x_derivativeAt(const double t) {
	return interpolateDerivative(x_values, t);
}

double InterpolationHelper::y_derivativeAt(const double t) {
	return interpolateDerivative(y_values, t);
}

double InterpolationHelper::interpolateValue(const alglib::real_1d_array &values, const double t) {

	alglib::ae_int_t info;
	alglib::barycentricinterpolant poly;
	alglib::polynomialfitreport rep;

	alglib::polynomialfit(timepoints, values, degree, info, poly, rep);

	return alglib::barycentriccalc(poly, t);
}

double InterpolationHelper::interpolateDerivative(const alglib::real_1d_array &values, const double t) {

	alglib::ae_int_t info;
	alglib::barycentricinterpolant poly;
	alglib::polynomialfitreport rep;
	alglib::polynomialfit(timepoints, values, degree, info, poly, rep);

	double f, df;
	alglib::barycentricdiff1(poly, t, f, df);

	return df;
}
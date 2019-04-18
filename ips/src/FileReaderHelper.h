#pragma once
#include "default.h"
#include "GlobalDataHelper.h"


class FileReaderHelper
{
public:

	//singelton
	static FileReaderHelper& getInstance()
	{
		static FileReaderHelper instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	~FileReaderHelper() {}

	/**
		find next point in string after pos ( find "[" "]")
	*/
	bool findNextPoint(const std::string &s, const size_t pos, std::string &point, size_t &end_point_pos);

	/**
		find next mapping in string after pos ( find "{" "}")
	*/
	bool findNextMapping(const std::string &s, const size_t pos, std::string &map, size_t &end_map_pos);

	/**
		find next char pair in string after pos (find c_1 c_2)
	*/
	bool findNextCharPair(const std::string &s, const std::string &c_1, const std::string &c_2, const size_t pos, std::string &result, size_t &end_result_pos);

	/**
		read world point from string
	*/
	WorldPoint getWorldPointFromString(const std::string &point);

	/**
		read world point from string when z value is given by a constant (e.g. VEHICLE_HEIGHT)
		instead of a double value
	*/
	WorldPoint getWorldPointWithConstantZFromString(const std::string &point);

	/**
		get double from string starting at pos
	*/
	double getDoubleFromString(const std::string &s, const size_t pos);

	/**
		get double from string starting at pos_1 and ending at pos_2
	*/
	double getDoubleFromString(const std::string &s, const size_t pos_1, const size_t pos_2);

	/**
		get int from string starting at pos
	*/
	int getIntFromString(const std::string &s, const size_t pos);

	/**
		get int from string starting at pos_1 and ending at pos_2
	*/
	int getIntFromString(const std::string &s, const size_t pos_1, const size_t pos_2);

	/**
		get geometrie from string
	*/
	void readGeometrieFromString(const std::string &s, Geometrie &geometrie);

	/**
		get interval from string
	*/
	Interval readIntervalFromString(const std::string &interval);

private:
	/**
		constrcutor only private
	*/
	FileReaderHelper() {}

	/**
		guarantees that no further instance can
		be constructed via copy-constructor
	*/
	FileReaderHelper(const FileReaderHelper&);

	/**
		avoids für instances with copy
	*/
	FileReaderHelper & operator = (const FileReaderHelper &);

};


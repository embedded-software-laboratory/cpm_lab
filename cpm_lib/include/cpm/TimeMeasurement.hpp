#pragma once

#include <string>
#include <memory>
#include <map>
#include <ctime>
#include "cpm/get_time_ns.hpp"


namespace cpm
{

    /**
     * \class MeasurementData
     * \brief Just for internal use.
     * 
     * Contains the data of one measurement.
     */
    class MeasurementData {
        public:
            //! Start time of measurement
            uint64_t start_time = 0;
            //! End time of measurement
            uint64_t end_time = 0;
            //! Linux clock used for both measurements
            clockid_t clockid;

            /**
             * \brief Constructor, which directly fixes the start time.
             * \param clockid ClockID which is used for both measurements.
             */
            MeasurementData(clockid_t clockid);
    };




    /**
     * \class TimeMeasurement
     * \brief This class comprises functions with which time measurements between specific points in the code can be made.
     *        One instance saves all measurements which are currently done (Singleton).
     */
    class TimeMeasurement {
        public:
            /**
             * \brief Provides access to the Singleton / creates it
             */
            static TimeMeasurement& Instance();

            // Neccessary deletion of functions when working with singleton pattern
            TimeMeasurement(TimeMeasurement const&) = delete;
            void operator=(TimeMeasurement const&)  = delete;

            /**
             * \brief This function starts a measurement. If there is already a measurment with this name the old data will be overriden.
             * \param name  The name of the measurement. All data for the measurment is saved under this name.
             * \param clockid The clockid which will be used for this measurement.
             */
            void start(std::string name, clockid_t clockid);

            /**
             * \brief Same as start above but uses default clockid.
             * \param name The name of the measurement. All data for the measurment is saved under this name.
             */
            void start(std::string name);

            /**
             * \brief This function stops a measurement.
             * \param name  The name of the measurement which is to be stopped.
             * \return      The result of the measurement.
             */
            uint64_t stop(std::string name);

            /**
             * \brief Returns all measurements in string format.
             */
            std::string get_str();

            /**
             * \brief Configure the clockid which is used as default when calling start() without specifying the clockid.
             * \param clockid Clockid of the corresponding clock under Linux.
             */
            void set_default_clockid(clockid_t clockid);

            /**
             * \brief Returns true iff a measurement with the given name exists (active or finished)
             * \param name Measurement name
             */
            bool exists(std::string name);


        private:
            /**
             * \brief Private constructor (due to Singleton)
             */
            TimeMeasurement(){}
            //! Singleton instance
            static TimeMeasurement& instance;

            //! Use monotonic clock as default, since we are interested in time differences and not in one correct timestamp
            clockid_t default_clockid = CLOCK_MONOTONIC;
            //! Map which saves all measurements according to their names.
            std::map<std::string, MeasurementData> measurements;

    };

} // namespace cpm

#pragma once

#include <stdexcept>

namespace cpm {
    /**
     * \brief Small error class for the timer
     * \ingroup cpmlib
     */
    class ErrorTimerStart: public std::runtime_error {    
    public:
        /**
         * \brief Constructor / Error type for timer error
         * \param msg Error message
         */
        ErrorTimerStart(const std::string& msg);
    };
}
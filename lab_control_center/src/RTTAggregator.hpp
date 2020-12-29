// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

#include <map>
#include <mutex>
#include <set>
#include <string>

#include "ReadyStatus.hpp"

#include "cpm/Logging.hpp"
#include "cpm/RTTTool.hpp"
#include "cpm/get_time_ns.hpp"

/**
 * \brief This class can be used to get RTT measurements and stop (/restart) them during simulation (/afterwards)
 * Measurements also include the number of missing replies (if no reply was received by any sender with the same ID (e.g. 'vehicle'))
 * After a timeout and no replies, measurements are reset / deleted for that ID (they are assumed to be turned off on purpose)
 * \ingroup lcc
 */
class RTTAggregator
{
private:
    //Calculated RTT values - one mutex for all, because we read / write on all at the same time
    std::mutex rtt_values_mutex;
    std::map<std::string, uint64_t> current_best_rtt;
    std::map<std::string, uint64_t> current_worst_rtt;
    std::map<std::string, uint64_t> all_time_worst_rtt;
    std::map<std::string, double> measure_count; //Starts counting as soon as the first answer was received, also counts missing ones
    std::map<std::string, double> missed_answers; //Total of missed answers, counting starts as for measure_count
    std::map<std::string, uint64_t> last_msg_timestamp;
    std::set<std::string> received_ids; //Remember IDs to notice missing ones

    //Timeout before entries are deleted again, because no answer was received for a longer time
    const uint64_t delete_entry_timeout_ns = 10e9; //10 seconds, because the RTT Tool already takes a while for one missing message

    //Thread for measuring the RTT regularly
    std::thread check_rtt_thread;
    std::atomic_bool run_rtt_thread;
    void create_rtt_thread();
    void destroy_rtt_thread();

    /**
     * \brief Used to delete an entry from all data structures
     * \param id Key of the entry
     */
    void delete_entry(std::string& id);

public:
    /**
     * \brief Measurement is started after object construction
     */
    RTTAggregator();
    ~RTTAggregator();

    /**
     * \brief Restart RTT measurement
     */
    void restart_measurement();

    /**
     * \brief Stop measurement of RTT (to reduce load on the network)
     */
    void stop_measurement();
    
    /**
     * \brief Get current measurements for RTT
     * \param participant_id ID of the participant of which the RTT was supposed to be measured (entry might not exist), e.g. 'vehicle'
     * \param current_best_rtt Best RTT of the current measurement (in ns)
     * \param current_worst_rtt Worst RTT of the current measurement (in ns)
     * \param all_time_worst_rtt Worst RTT of all measurements (in ns)
     * \param missed_answers Percentage of missed answers for this participant (Measured from first received answer)
     * \returns False if no RTT has yet been measured, else true
     */
    bool get_rtt(std::string participant_id, uint64_t &_current_best_rtt, uint64_t &_current_worst_rtt, uint64_t &_all_time_worst_rtt, double &_missed_answers);
};
#pragma once

#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "ReadyStatus.hpp"

#include "cpm/ParticipantSingleton.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/AsyncReader.hpp"

/**
 * \brief This class collects all the ReadySignal messages sent by the HLCs (NUCs) to present the GUI user the NUCs (/IDs) that are currently online
 * This class can also be used to just retrieve the currently available IDs for distribution of scripts on several NUCs (TODO/WIP)
 */
class HLCReadyAggregator
{
private:
    //Reader, mutex and list to get and store a list of currently online HLCs
    //A map is used because each ID has a time to live, which is updated whenever a new sample with this ID is received - this is supposed to handle NUC crashes (-> not shown to be online anymore)
    cpm::AsyncReader<ReadyStatus> async_hlc_reader;
    std::mutex hlc_list_mutex;
    std::map<std::string, uint64_t> hlc_map;

    //The HLCs send a signal every second, so they are probably offline if no signal was received within 3 seconds
    const uint64_t time_to_live_ns = 3000000000;

public:
    HLCReadyAggregator();
    std::vector<std::string> get_hlc_ids_string();
    std::vector<uint8_t> get_hlc_ids_uint8_t();
};
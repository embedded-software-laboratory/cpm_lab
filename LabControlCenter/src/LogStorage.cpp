#include "LogStorage.hpp"

using namespace std::placeholders;
LogStorage::LogStorage() :
    /*Set up communication*/
    log_reader(std::bind(&LogStorage::log_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<Log>("Logs"), true)
{    
}

void LogStorage::log_callback(dds::sub::LoanedSamples<Log>& samples) {    
    for (auto sample : samples) {
        if (sample.info().valid()) {
            std::lock_guard<std::mutex> lock_1(log_storage_mutex);
            std::lock_guard<std::mutex> lock_2(log_buffer_mutex);
            log_storage.push_back(sample.data());
            log_buffer.push_back(sample.data());
        }
    }

    //TODO: Clear storage and buffer when some max size was reached
}

std::vector<Log> LogStorage::get_new_logs() {
    std::lock_guard<std::mutex> lock(log_buffer_mutex);
    std::vector<Log> log_copy = log_buffer;
    log_buffer.clear();
    return log_copy;
}

std::vector<Log> LogStorage::perform_abortable_search(std::string filter_value, FilterType filter_type, std::atomic_bool &continue_search) {
    //TODO: Does this lock too long? Are logs lost because they cannot be added to the storage?
    //First idea to change this: Copy log_storage and perform search on copy only
    std::lock_guard<std::mutex> lock(log_storage_mutex);
    std::vector<Log> search_result;

    for (auto iterator = log_storage.begin(); iterator != log_storage.end(); ++iterator) {
        if (!continue_search.load()) {
            break;
        }

        std::stringstream stream;
        std::string filter_by_text;
        switch (filter_type) {
            case ID:
                filter_by_text = iterator->id();
                break;
            case Content:
                filter_by_text = iterator->content();
                break;
            case Timestamp:
                stream << iterator->stamp().nanoseconds();
                filter_by_text = stream.str();
                break;
            default:
                stream << iterator->id() << iterator->content() << iterator->stamp().nanoseconds();
                filter_by_text = stream.str();
                break;
        }

        if (filter_by_text.find(filter_value) != string::npos) {
            search_result.emplace_back(*iterator);
        }
    }

    return search_result;
}
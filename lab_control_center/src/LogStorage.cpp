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

#include "LogStorage.hpp"

using namespace std::placeholders;
LogStorage::LogStorage() :
    /*Set up communication*/
    log_reader(std::bind(&LogStorage::log_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<Log>("log"), true)
{    
    file.open(filename, std::ofstream::out | std::ofstream::trunc);
    file << "ID,Timestamp,Content" << std::endl;
}

LogStorage::~LogStorage()
{
    //Close the file in the destructor
    file.close();
}

void LogStorage::log_callback(dds::sub::LoanedSamples<Log>& samples) { 
    std::lock_guard<std::mutex> lock_1(log_storage_mutex);
    std::lock_guard<std::mutex> lock_2(log_buffer_mutex); 

    for (auto sample : samples) {
        if (sample.info().valid()) {
            //Make sure that the utf8-encoding is correct, or else Gtk will show a warning (Pango, regarding UTF-8)
            //The warning will still show up, but the log message is altered s.t. the user can find the error
            Log received_log = sample.data();
            assert_utf8_validity(received_log);

            log_storage.push_back(received_log);
            log_buffer.push_back(received_log);

            //Write logs immediately to csv file (taken from cpm library)
            //For the log file: csv, so escape '"'
            std::string str = received_log.content();
            std::string log_string = std::string(str);
            std::string escaped_quote = std::string("\"\"");
            size_t pos = 0;
            while ((pos = log_string.find('"', pos)) != std::string::npos) {
                log_string.replace(pos, 1, escaped_quote);
                pos += escaped_quote.size();
            }
            //Also put the whole string in quotes
            log_string.insert(0, "\"");
            log_string += "\"";

            //Mutex for writing the message (file, writer) - is released when going out of scope
            std::lock_guard<std::mutex> lock(file_mutex);

            //Add the message to the log file
            file << received_log.id() << "," << received_log.stamp().nanoseconds() << "," << log_string << std::endl;
        }
    }

    //Clear storage and buffer when some max size was reached - keep last elements
    keep_last_elements(log_storage, 10000);
    keep_last_elements(log_buffer, 100);
}

std::vector<Log> LogStorage::get_new_logs(unsigned int log_level) {
    std::lock_guard<std::mutex> lock(log_buffer_mutex);
    std::vector<Log> log_copy;

    for (auto log : log_buffer)
    {
        //TODO: It could be more efficient to store logs of different levels in different data structures (though merging them would take more time)
        if(log.log_level() <= log_level)
        {
            log_copy.push_back(log);
        }
    }

    log_buffer.clear();
    return log_copy;
}

std::vector<Log> LogStorage::get_all_logs(unsigned short log_level) {
    std::lock_guard<std::mutex> lock(log_storage_mutex);
    std::vector<Log> log_copy;

    for (auto log : log_storage)
    {
        //TODO: It could be more efficient to store logs of different levels in different data structures (though merging them would take more time)
        if(log.log_level() <= log_level)
        {
            log_copy.push_back(log);
        }
    }

    return log_copy;
}

std::vector<Log> LogStorage::get_recent_logs(const long log_amount, unsigned short log_level) {
    std::lock_guard<std::mutex> lock(log_storage_mutex);
    std::vector<Log> log_copy;
    long count = 0;
    for (auto back_it = log_storage.rbegin(); back_it != log_storage.rend(); ++back_it)
    {
        //Only add log_amount most recent logs
        if (count >= log_amount) break;

        //Get next log, if it is within the given log level
        if (back_it->log_level() <= log_level)
        {
            log_copy.push_back(*back_it);
            ++count;
        }
    }

    return log_copy;
}

std::vector<Log> LogStorage::perform_abortable_search(std::string filter_value, FilterType filter_type, unsigned short log_level, std::atomic_bool &continue_search) {
    //Copy log_storage and perform search on copy only
    std::unique_lock<std::mutex> lock(log_storage_mutex);
    std::vector<Log> log_storage_copy = std::vector<Log>(log_storage);
    lock.unlock();

    //Result vector
    std::vector<Log> search_result;

    try {
        std::regex search_regex(filter_value);

        for (auto iterator = log_storage_copy.begin(); iterator != log_storage_copy.end(); ++iterator) {
            if (!continue_search.load()) {
                //Return empty result list if the search was aborted
                search_result.clear();
                return search_result;
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

            if (std::regex_search(filter_by_text, search_regex) && iterator->log_level() <= log_level) {
                search_result.emplace_back(*iterator);
            }

            // if (filter_by_text.find(filter_value) != string::npos) {
            //     search_result.emplace_back(*iterator);
            // }
        }
    }
    catch (std::regex_error& e) {
        std::cout << "Regex error (due to filter string): " << e.what() << std::endl;
        search_result.push_back(Log("", "No results - Wrong regex expression!", TimeStamp(0), 0));
    }

    if (search_result.size() == 0 && log_storage_copy.size() > 0) {
        search_result.push_back(Log("", "No results", TimeStamp(0), 0));
    }

    keep_last_elements(search_result, 100);

    return search_result;
}

void LogStorage::keep_last_elements(std::vector<Log>& vector, size_t count) {
    //Does not use lock() because it is supposed to be called from a function where lock() has been called before
    if (vector.size() > count) {
        vector.erase(vector.begin(), vector.end() - count);
    }
}

void LogStorage::assert_utf8_validity(Log& log)
{
    std::string log_id_string(log.id());
    std::string log_msg_string(log.content());

    //Use a glib function to check utf8 conformance
    bool log_msg_valid = g_utf8_validate(log_msg_string.c_str(), -1, NULL);
    bool log_id_valid = g_utf8_validate(log_id_string.c_str(), -1, NULL);

    //If non-conformant, append an error message to the log string, s.t. this message shows up in the UI (Logs)
    if (! (log_msg_valid && log_id_valid))
    {
        std::stringstream changed_log_message_stream;
        changed_log_message_stream << "INVALID UTF-8 ENCODING IN: ";

        if (! log_id_valid)
        {
            changed_log_message_stream << " ID | ";
        }
        if (! log_msg_valid)
        {
            changed_log_message_stream << " CONTENT | ";
        }

        changed_log_message_stream << " --- msg was: " << log_msg_string;

        log.content(changed_log_message_stream.str());
    }
}

void LogStorage::reset() 
{
    std::unique_lock<std::mutex> lock(log_storage_mutex);
    std::unique_lock<std::mutex> lock_2(log_buffer_mutex);
    log_storage.clear();
    log_buffer.clear();

    //Reset UI file
    file.clear();
}
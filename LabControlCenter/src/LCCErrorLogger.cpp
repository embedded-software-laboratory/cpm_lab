#include "LCCErrorLogger.hpp"

LCCErrorLogger& LCCErrorLogger::Instance()
{
    static LCCErrorLogger instance;
    return instance;
}

void LCCErrorLogger::log_error(std::string error)
{
    std::lock_guard<std::mutex> lock(error_storage_mutex);
    std::lock_guard<std::mutex> lock2(new_error_storage_mutex);

    //String is only new if it has not been added before
    if (error_storage.find(error) == error_storage.end())
    {
        new_error_storage.insert(error);
        error_storage.insert(error);
    }
}

std::unordered_set<std::string> LCCErrorLogger::get_all_errors()
{
    std::lock_guard<std::mutex> lock(error_storage_mutex);

    return error_storage;
}

std::unordered_set<std::string> LCCErrorLogger::get_new_errors()
{
    std::lock_guard<std::mutex> lock(new_error_storage_mutex);

    auto new_error_storage_copy = new_error_storage;
    new_error_storage.clear();

    return new_error_storage_copy;
}

void LCCErrorLogger::reset()
{
    std::lock_guard<std::mutex> lock(error_storage_mutex);
    std::lock_guard<std::mutex> lock2(new_error_storage_mutex);

    error_storage.clear();
    new_error_storage.clear();
}
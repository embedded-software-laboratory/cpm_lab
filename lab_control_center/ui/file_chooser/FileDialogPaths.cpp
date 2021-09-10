#include "FileDialogPaths.hpp"

FileDialogPaths::FileDialogPaths()
{
    //Create the file chooser config file, if it does not already exist
    std::experimental::filesystem::path filepath = config_location;
    bool file_exists = std::experimental::filesystem::exists(filepath);
    if (!file_exists)
    {
        //Create file
        std::ofstream yaml_file(config_location);
        yaml_file.close();
    }
}

FileDialogPaths& FileDialogPaths::Instance() {
    // Thread-safe in C++11
    static FileDialogPaths myInstance;

    return myInstance;
}

std::string FileDialogPaths::get_last_execution_path(std::string config_name)
{
    std::lock_guard<std::mutex> lock(config_file_mutex);

    auto yaml_config_profile = YAML::LoadFile(config_location);
    std::string return_path = "";

    std::string error_string;

    //Behaviour if no last execution path could be found - set to default folder
    //Also: In case the yaml profile is malformed, reset
    try 
    {
        if (! yaml_config_profile[config_name])
        {
            return_path = default_load_path;
        }
        else
        {
            return_path = yaml_config_profile[config_name].as<std::string>();
        }
    }
    catch (const std::domain_error& err)
    {
        error_string = err.what();
    }
    catch (const std::exception& err)
    {
        error_string = err.what();
    }

    //In case of an error, reset the file
    if (error_string.size() > 0)
    {
        std::cout << "NOTE: Resetting YAML file dialog config, is malformed" << std::endl;
        std::ofstream yaml_file;
        yaml_file.open(config_location, std::ofstream::out | std::ofstream::trunc);
        yaml_file.close();

        return_path = default_load_path;
    }

    return return_path;
}

void FileDialogPaths::store_last_execution_path(std::string filename, std::string config_name)
{
    std::lock_guard<std::mutex> lock(config_file_mutex);

    //Load current profile
    auto yaml_config_profile = YAML::LoadFile(config_location);

    //Store change in profile, reset and store again in case of error
    std::string error_string;
    try 
    {
        yaml_config_profile[config_name] = filename;
    }
    catch (const std::domain_error& err)
    {
        error_string = err.what();
    }
    catch (const std::exception& err)
    {
        error_string = err.what();
    }

    //In case of an error, reset the file
    if (error_string.size() > 0)
    {
        std::cout << "NOTE: Resetting YAML file dialog config, is malformed" << std::endl;
        std::ofstream yaml_file;
        yaml_file.open(config_location, std::ofstream::out | std::ofstream::trunc);
        yaml_file.close();

        yaml_config_profile = YAML::LoadFile(config_location);
        yaml_config_profile[config_name] = filename;
    }

    //Store changed profile
    std::ofstream yaml_file(config_location, std::ofstream::out | std::ofstream::trunc);
    yaml_file << yaml_config_profile;
    yaml_file.close();
}
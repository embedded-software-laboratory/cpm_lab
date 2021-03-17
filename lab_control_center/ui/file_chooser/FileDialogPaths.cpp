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

    //Behaviour if no last execution path could be found - set to HLC folder
    if (! yaml_config_profile[config_name])
    {
        return_path = default_load_path;
    }
    else
    {
        return_path = yaml_config_profile[config_name].as<std::string>();
    }
    

    return return_path;
}

void FileDialogPaths::store_last_execution_path(std::string filename, std::string config_name)
{
    std::lock_guard<std::mutex> lock(config_file_mutex);

    //Load current profile
    auto yaml_config_profile = YAML::LoadFile(config_location);
    yaml_config_profile[config_name] = filename;

    //Store changed profile
    std::ofstream yaml_file(config_location, std::ofstream::out | std::ofstream::trunc);
    yaml_file << yaml_config_profile;
    yaml_file.close();
}
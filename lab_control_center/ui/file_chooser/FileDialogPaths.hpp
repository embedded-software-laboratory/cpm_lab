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

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <fstream>
#include <iostream>
#include <locale>
#include <string>

//For storing last chosen file(s)
#include <yaml-cpp/yaml.h>
#include <experimental/filesystem>
#include <mutex>

/**
 * \class FileDialogPaths
 * \brief This class is used both by FileChooserUI and FileSaverUI.
 * Both need to store / load file paths previously selected by the user, so that they do not open on the home directory 
 * but remember the last chosen file.
 * This class is a Singleton.
 * \ingroup lcc_ui
 */
class FileDialogPaths {
public:
    //! Singleton instance
    static FileDialogPaths& Instance();

    //Delete move and copy op
    FileDialogPaths(FileDialogPaths const&) = delete;
    FileDialogPaths(FileDialogPaths&&) = delete; 
    FileDialogPaths& operator=(FileDialogPaths const&) = delete;
    FileDialogPaths& operator=(FileDialogPaths &&) = delete;

    /**
     * \brief Function to store the last execution / loaded file path for the set config name in the config file
     * \param filename The path to store
     * \param config_name In a configuration file, previous file locations are stored, for the convenience of the user. The config name, if not default, can be used to remember the last file for this specific use-case (e.g. for parameters).
     */
    void store_last_execution_path(std::string filename, std::string config_name = "default");

    /**
     * \brief Returns the previously selected path of last program execution
     * \param config_name In a configuration file, previous file locations are stored, for the convenience of the user. The config name, if not default, can be used to remember the last file for this specific use-case (e.g. for parameters).
     */
    std::string get_last_execution_path(std::string config_name = "default");
private:
    /**
     * \brief Constructor.
     */
    FileDialogPaths();

    //! Default load path if no previous file is present
    const std::string default_load_path = "";
    //! Location of the config file for this file chooser, which tells the previously selected file of the last program execution
    const std::string config_location = "./file_dialog_open_config.yaml";

    //! To make sure that only one file chooser can access the config file at a time
    std::mutex config_file_mutex;
};
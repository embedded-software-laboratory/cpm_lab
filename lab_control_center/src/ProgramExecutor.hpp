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

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

//To spawn a process & get its PID
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * \brief Issue: Fork in multi-threaded programs can lead to problems, e.g. when malloc is used, 
 * and we do not have full control over all used libraries (e.g. for DDS).
 * 
 * Solution: Fork before all other parts of the LCC executable are initalized. This gives us a child
 * process with minimal content, created when no threads are running. This child process can then be
 * communicated with to fork from it to execute other programs, like the labcam, simulated vehicles 
 * etc. This class bundles the creation of this child with messaging, to let the child create further
 * child processes to execute other programs. Its destructor also makes sure that (in case the LCC
 * is terminated normally) the child is waited for after destruction, so that it does not remain as
 * a zombie process. The same goes for the message queue.
 * 
 * \ingroup lcc
 */
class ProgramExecutor {
public:
    /**
     * \brief Destructor; kill and wait for the child process, destroy the message queue
     */
    ~ProgramExecutor();

    /**
     * \brief Needs to be called after the constructor; sets up the child process (by calling fork()) and the message queue for communication.
     * DO NOT call this unless you know that your program is currently safe to fork (e.g. at the start of your main).
     * PLEASE USE the return value, which tells the program if it is the child or parent process.
     * 
     * Reasoning: The behaviour for setup is different for the child and the main process. This behaviour should not be split in the constructor.
     * 
     * \param filepath A unique filepath. You should use argv[0]. Is required for message queue creation.
     * 
     * \return True if parent process, false if the execution failed. (Exits before return in case of child process)
     */
    bool setup_child_process(std::string filepath);

    /**
     * \brief Call this function to start an external program, like a simulated vehicle etc.
     * You can use commands that would usually work in the terminal, like tmux commands.
     * If no timeout is set, the command is started in the child using system(). Else, we
     * fork explicitly, and kill the process after the timeout. During that time, other
     * executions are "on hold" (no threads are used to allow to execute multiple commands
     * with a timeout at once). Thus, commands can "pile up" in the msg queue for a while,
     * but should finally be executed nonetheless.
     * \param command String command, like a command line command. Warning: the command length is limited, as a char array is used in the message queue.
     * \param timeout Time after which the command should be cancelled, if it is not finished before that. If not set or -1, no timeout is used.
     */
    void execute_command(std::string command, int timeout = -1);

private:
    /**
     * \brief Represents the current state of a running process
     */
    enum PROCESS_STATE {
        RUNNING, ERROR, DONE
    };

    //! Communication via message queues requires a communication struct
    struct CommandMsg {
        //! Always required, can be used as and ID to get a sepcific msg (won't be used here though)
        long mtype;

        //! Better to use only one additional entry, so if more than one is required, define another struct here
        struct CommandInfo {
            char command[5000];
            int timeout_seconds;
        } command;
    };

    //! The process ID of the child, needs to be remembered to wait for it in the destructor
    pid_t child_process_id;

    //! The ID of the message queue, needs to be remembered to use the queue and to close it in the destructor
    int msg_queue_id;

    /**
     * \brief Create a CommandMsg from a command string and a given timeout. Returns false if creation failed 
     * (usually: due to too large command_string, which is limited by the size of command in CommandInfo)
     * \param command_string The command to execute 
     * \param timeout_seconds Timeout for the command, set to a value <0 to disable timeouts for this command 
     *                      (SYSTEM is used then instead of using fork explicitly)
     */
    bool create_msg(std::string command_string, int timeout_seconds, CommandMsg& command_out);

    /**
     * \brief Send the desired message on the given message queue. Prints an error and returns false if it failed, else returns true
     * \param msqid ID of the msg queue to use
     * \param msg Message to send
     */
    bool send_msg(int msqid, CommandMsg& msg);

    /**
     * \brief Receive a message on the given message queue. Prints an error and returns false if it failed, else returns true
     * \param msqid ID of the msg queue to use
     * \param msg The received msg is stored here
     */
    bool receive_msg(int msqid, CommandMsg& msg);

    /**
     * \brief Function to execute a shell command and get its output
     * \param cmd A shell command as C-String
     * \return Output of the shell command
     */
    std::string execute_command_get_output(const char* cmd);

    /**
     * \brief Check if the given tmux session already exists - used to kill left-over sessions
     * \return True if the session exists, false otherwise
     */
    bool session_exists(std::string session_id);

    /**
     * \brief Kill a tmux session with the given session_id - only if it exists (uses session_exists)
     */
    void kill_session(std::string session_id);

    /**
     * \brief Creates a command and manages it until it finished or a timeout occured or the HLC is no longer online; uses the three functions below
     * \param cmd Command string to be executed
     * \param timeout_seconds Timout until the process termination is forced
     * \param is_online Optional. Function to check whether the spawned process is still running.
     * \return True if the execution (of the bash script) did not have to be aborted and no process-related error occured, false otherwise 
     */
    bool spawn_and_manage_process(const char* cmd, unsigned int timeout_seconds, std::function<bool()> is_online = [] { return true; });

    /**
     * \brief Function to execute a shell command that returns the processes PID, so that the process can be controlled / monitored further
     * \param cmd A shell command as C-String
     * \return Output of the shell command
     */
    int execute_command_get_pid(const char* cmd);

    /**
     * \brief Function to find out which state a process spawned before is currently in
     * \param process_id The process id of the child process that was spawned before
     * \return The current state of the process
     */
    PROCESS_STATE get_child_process_state(int process_id);

    /**
     * \brief Kill a process - first allow it to terminate gracefully, else force-kill it
     * \param process_id The ID of the process
     * \param timeout_ms Time to wait between SIGTERM and SIGKILL (in case SIGTERM did not succeed until the timeout happened)
     */
    void kill_process(int process_id, int timeout_ms = 3000);

};
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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
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
 * NOTE: This class CANNOT use the CPM Logger or any other CPM function. If it does, the above errors
 * are not resolved and the class becomes meaningless!
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
     * \param filepath_1 A unique filepath. Is required for message queue creation.
     * \param filepath_2 Another unique filepath. Is required for message queue creation.
     * 
     * \return True if parent process, false if the execution failed. (Exits before return in case of child process)
     */
    bool setup_child_process(std::string filepath_1, std::string filepath_2);

    /**
     * \brief Call this function to start an external program, like a simulated vehicle etc.
     * You can use commands that would usually work in the terminal, like tmux commands.
     * If no timeout is set, the command is started in the child using system(). Else, we
     * fork explicitly, and kill the process after the timeout. During that time, other
     * executions are "on hold" (no threads are used to allow to execute multiple commands
     * with a timeout at once). Thus, commands can "pile up" in the msg queue for a while,
     * but should finally be executed nonetheless.
     * 
     * If a timeout is set, the function also returns whether the executed command succeeded (= finished in time).
     * It also returns false if the command could not be sent via the IPC msg queue, either due to an internal failure or if the 
     * command string was too large.
     * \param command String command, like a command line command. Warning: the command length is limited, as a char array of 5000 chars is used in the message queue.
     * \param timeout Time after which the command should be cancelled, if it is not finished before that. If not set or -1, no timeout is used.
     * \return True if the timeout is > 0. Else true if the command worked, false if it could not be sent, failed or reached the given timeout before it finished.
     */
    bool execute_command(std::string command, int timeout = -1);

    /**
     * \brief Function to execute a shell command and get its output
     * \param cmd A shell command
     * \return Output of the shell command
     */
    std::string get_command_output(std::string cmd);

private:
    /**
     * \brief Represents the current state of a running process
     */
    enum PROCESS_STATE {
        RUNNING, ERROR, DONE
    };

    /**
     * \brief Represents the command request type - if an answer by the child is expected or not
     */
    enum RequestType {
        NO_ANSWER, SEND_OUTPUT, SEND_EXIT_STATE
    };

    //! Communication via message queues requires a communication struct. Here: Sending commands to the child process.
    struct CommandMsg {
        //! Always required, can be used as and ID to get a sepcific msg (won't be used here though)
        long mtype;

        //! Better to use only one additional entry, so if more than one is required, define another struct here
        struct CommandInfo {
            //! The command string to execute. Max msg size on linux may only be 8192 bytes.
            char command[5000];

            //! Timeout for the command, ignored if <= 0
            int timeout_seconds;

            //! What kind of answer is expected by the child via the msg_receive_queue
            RequestType request_type;
        } command;
    };

    //! Communication via message queues requires a communication struct. Here: Receiveing answers from the child process.
    struct AnswerMsg {
        //! Always required, can be used as and ID to get a sepcific msg (won't be used here though)
        long mtype;

        //! Better to use only one additional entry, so if more than one is required, define another struct here
        struct AnswerInfo {
            //! The output of a command, truncated. Max msg size on linux may only be 8192 bytes.
            char truncated_command_output[5000];

            //! Process state of the process if timeout was used and process state was asked for, true if DONE, else false
            bool execution_success;
        } answer;
    };

    //! The process ID of the child, needs to be remembered to wait for it in the destructor
    pid_t child_process_id;

    //! The ID of the message queue, needs to be remembered to use the queue and to close it in the destructor
    int msg_request_queue_id;

    //! The ID of the message queue for answers sent by the child process
    int msg_response_queue_id;

    //Variables for the child's thread pool
    //! Amount of threads for parallel execution
    const int thread_pool_size = 6;
    //! Holds all worker threads
    std::vector<std::thread> thread_pool;
    //! Holds all open commands
    std::queue<CommandMsg> job_queue;
    //! Access to the job_queue
    std::mutex job_mutex;
    //! Wakes up waiting threads if queue was empty and now a new job was put into it
    std::condition_variable jobs_available;
    //! To stop all running threads and exit the child process afterwards
    std::atomic_bool stop_threads {false};

    //! Counter for current command ID - NEVER access this directly, ALWAYS use the increment function
    long current_command_id = 1;
    //! To get access to the current command id
    std::mutex command_id_mutex;


    //! File to log "main" child error(s) to
    std::string filename;
    //! Mutex for logging
    std::mutex log_mutex;

    /**
     * \brief Log an (error) msg to a file, unique w.r.t. the starting time of the LCC
     * We cannot use the cpm Logger here, as it uses DDS / its own threads. 
     * File name is determined by start time of LCC and differs from LCC Error Logger Name (on purpose).
     * 
     * ONLY USED BY THE CHILD PROCESS that manages all other processes (and by the parent process, but only if child init. fails)
     * 
     * \param message Message to append to the log file
     */
    void log(std::string message);

    /**
     * \brief To create a unique log ID, uses cpm Log constructor code, as we cannot use the cpm lib here
     */
    std::string get_unique_log_id();

    /**
     * \brief You need a command ID for your commands. Always use this function and nothing else to obtain one! (Is thread-safe)
     */
    long get_unique_command_id();

    /**
     * \brief (Child) Add a job to the job queue for the thread pool to execute it
     * \param msg Job / command received from the parent
     */
    void add_job(CommandMsg& msg);

    /**
     * \brief Function executed by all worker threads
     */
    void do_jobs_or_stop();

    /**
     * \brief To kill all running threads and destroy the thread pool
     */
    void stop_thread_pool();

    /**
     * \brief Helper for the child process. Processes a single command msg.
     * \param msg The received command msg.
     */
    void process_single_child_command(CommandMsg& msg);

    /**
     * \brief Creates a msg queue and returns its ID
     * \param filepath Required to create the queue key
     */
    int create_msg_queue(std::string filepath);

    /**
     * \brief Destroy a msg queue, given its ID
     * \param msg_queue_id The ID of the msg queue to destroy
     */
    void destroy_msg_queue(int msg_queue_id);

    /**
     * \brief Create a CommandMsg from a command string and a given timeout. Returns false if creation failed 
     * (usually: due to too large command_string, which is limited by the size of command in CommandInfo)
     * 
     * Not all combinations of commands are implemented, e.g. if the command output should be printed, the timeout will be ignored!
     * This is due to the implementation, which uses popen.
     * 
     * \param command_string The command to execute 
     * \param command_out The created command message object / output
     * \param command_id Unique ID of the command, to be able to retrieve answers / responses by the child process
     * \param timeout_seconds Optional: Timeout for the command, set to a value <0 to disable timeouts for this command 
     *                      (SYSTEM is used then instead of using fork explicitly)
     * \param request_type Optional: If an answer containing the command output or the child exit state should be returned via msg queue
     */
    bool create_command_msg(std::string command_string, CommandMsg& command_out, long command_id, int timeout_seconds = -1, RequestType request_type = NO_ANSWER);

    /**
     * \brief Send the desired message on the given message queue. Prints an error and returns false if it failed, else returns true.
     * Note: As far as I know, msgsnd is thread-safe, so no mutexes are used here.
     * \param msqid ID of the msg queue to use
     * \param msg Message to send
     */
    bool send_command_msg(int msqid, CommandMsg& msg);

    /**
     * \brief Receive a message on the given message queue. Prints an error and returns false if it failed, else returns true.
     * Note: As far as I know, msgrcv is thread-safe, so no mutexes are used here (And different mtypes are used -> no race condition).
     * 
     * ONLY TO BE USED BY THE CHILD PROCESS
     * \param msqid ID of the msg queue to use
     * \param msg The received msg is stored here
     */
    bool receive_command_msg(int msqid, CommandMsg& msg);

    /**
     * \brief Create and send the desired message on the given message queue. Prints an error and returns false if it failed, else returns true.
     * Note: As far as I know, msgsnd is thread-safe, so no mutexes are used here.
     * 
     * ONLY TO BE USED BY THE CHILD PROCESS
     * \param msqid ID of the msg queue to use
     * \param command_output Command output to send, may get truncated (max. 5000 characters)
     * \param command_id Unique ID of the command, to be able to retrieve answers / responses by the child process for the right request
     * \param execution_success Exit state of the process, success if DONE (no ERROR or timeout)
     */
    bool send_answer_msg(int msqid, std::string command_output, long command_id, bool execution_success);

    /**
     * \brief Receive a message on the given message queue. Prints an error and returns false if it failed, else returns true.
     * Note: As far as I know, msgrcv is thread-safe, so no mutexes are used here (And different mtypes are used -> no race condition).
     * \param msqid ID of the msg queue to use
     * \param command_id ID of the answer, is the same as the ID of the sent request
     * \param msg The received msg is stored here
     */
    bool receive_answer_msg(int msqid, long command_id, AnswerMsg& msg);

    /**
     * \brief Function to execute a shell command and get its output.
     * Is currently not stopped if stop_threads becomes true. 
     * But: A timeout of max. 10 seconds is always added to the command in the executing child 
     * due to troubles with program errors before.
     * \param cmd A shell command as C-String
     * \return Output of the shell command, or ERROR in case of a serious error or command timeout
     */
    std::string execute_command_get_output(const char* cmd);

    /**
     * \brief Creates a command and manages it until it finished or a timeout occured or the HLC is no longer online; uses the three functions below.
     * Also uses stop_threads during waiting, so a process spawned this way can be killed if the child should terminate all executions.
     * \param cmd Command string to be executed
     * \param timeout_seconds Timout until the process termination is forced
     * \return True if the execution (of the bash script) did not have to be aborted and no process-related error occured, false otherwise 
     */
    bool spawn_and_manage_process(const char* cmd, unsigned int timeout_seconds);

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

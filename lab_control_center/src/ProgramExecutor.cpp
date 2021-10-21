#include "ProgramExecutor.hpp"

/**
 * \file ProgramExecutor.cpp
 * \ingroup lcc
 */

ProgramExecutor::~ProgramExecutor()
{
    //Do something if not the child
    if (child_process_id != 0)
    {
        //Send final msg to tell the child to stop its execution
        //Only do this if child process creation was successful
        if (child_process_id > 0)
        {
            CommandMsg msg;
            if (create_command_msg("EXIT", msg, get_unique_command_id()))
            {
                send_command_msg(msg_request_queue_id, msg);
            }
            else
            {
                std::cerr << "ERROR: Could not send stop signal via IPC Message Queue, command string was too large" << std::endl;
            }

            //----------------------------------------CLEANING UP-----------------------------------------------------------//
            //The parent ALWAYS has to wait for the child to finish, else ressources are not cleaned up
            //There is an alternative to that (ignore SIGCHLD), but as far as I interpret it that would probably lead to 
            //problems with get_child_process_state
            kill_process(child_process_id, 100);
        }

        //Destroy the message queue
        destroy_msg_queue(msg_request_queue_id);
        destroy_msg_queue(msg_response_queue_id);
    }
    else if (child_process_id == 0)
    {
        //Release threads in the child process, if they already have been created
        stop_thread_pool();
    }
}

bool ProgramExecutor::setup_child_process(std::string filepath_1, std::string filepath_2)
{
    //Set the filename for the log file
    //It should only be accessed by the parent if the child does not (yet) exist
    filename = "Log_child_process_";
    filename += get_unique_log_id();
    filename += ".csv";

    //Create log head
    std::ofstream file;
    file.open(filename, std::ofstream::out | std::ofstream::trunc);
    file << "Timestamp of child error msg,Error msg" << std::endl;
    file.close();

    //Create msg queues (must be done before forking, so that both processes have a working reference)
    //Uses log() because it is called before fork()
    msg_request_queue_id = create_msg_queue(filepath_1);
    msg_response_queue_id = create_msg_queue(filepath_2);

    //Create the child process
    child_process_id = fork();

    if (child_process_id == 0)
    {
        //Tell the child to set its group process ID to its process ID, or else things like kill(-pid) to kill a ping-while-loop won't work
        setpgid(0, 0);

        //Set up the thread pool
        for (int i = 0; i < thread_pool_size; ++i)
        {
            thread_pool.push_back(
                std::thread([this] () {
                    do_jobs_or_stop();
                })
            );
        }

        //Now try to receive messages and add them to the task queue; the child waits if there currently is no message present
        //0 to receive the next message on the queue, irrelevant of the value of mtype, flag not used (0 as well)
        while (true)
        {
            //Receive_command_msg blocks until a new command is received
            CommandMsg msg;
            bool receive_success = receive_command_msg(msg_request_queue_id, msg);

            //Exit condition - just compare the first characters, as strncpy fills up the char array
            std::string msg_string = msg.command.command;
            if (msg_string.compare(0, 4, "EXIT") == 0)
            {
                stop_thread_pool();
                break;
            }

            //Add the task to the task queue
            if (receive_success)
                add_job(msg);
        }

        std::cout << "Child is stopping execution..." << std::endl;

        exit(EXIT_SUCCESS); //Should be okay, as this is only called in the child process
    }
    else if (child_process_id > 0)
    {
        //We are in the parent process and can now proceed in main
        return true;
    }
    else 
    {
        //We could not spawn a new process - usually, the program should not just break at this point, unless that behaviour is desired
        log("Error when using fork: Could not create 'meta'-child process! Without it, no external program can run, so the LCC is shut down now...");
        return false;
    }
}

void ProgramExecutor::log(std::string message)
{
    //For the log file: csv, so escape '"'
    std::string log_string(message);
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
    std::lock_guard<std::mutex> lock(log_mutex);

    //First print the message using std::cerr
    std::cerr << "ERROR IN CHILD: " << message << std::endl;

    //Then print the message to the current log file
    std::ofstream file;
    file.open(filename, std::ios::app);
    file << get_unique_log_id() << "," << log_string << std::endl;
    file.close();
}

std::string ProgramExecutor::get_unique_log_id() {
    // Formatted start time for log filename
    char time_format_buffer[100];
    {
        struct tm* tm_info;
        time_t timer;
        time(&timer);
        tm_info = gmtime(&timer);
        strftime(time_format_buffer, 100, "%Y_%m_%d_%H_%M_%S", tm_info);
    }

    std::string ret(time_format_buffer);
    return ret;
}

long ProgramExecutor::get_unique_command_id()
{
    //Get access to the current command id
    std::lock_guard<std::mutex> lock(command_id_mutex);
    auto id = current_command_id;

    //Increment the command ID. We do not need that high numbers (we should never have more than 20 or so tasks at once), so just start re-counting at 1e5
    if (current_command_id > 1e5)
    {
        current_command_id = 1;
    }
    else
    {
        ++current_command_id;
    }

    //Return the currently "unique" ID
    return id;
}

void ProgramExecutor::add_job(CommandMsg& msg)
{
    //Acquire the queue mutex
    std::unique_lock<std::mutex> lock(job_mutex);

    //Add the job to the queue
    job_queue.push(msg);
    lock.unlock();

    //Now, one thread is woken up and picks up the new job (if it had to wait on a formerly empty queue)
    jobs_available.notify_one();
}

void ProgramExecutor::do_jobs_or_stop()
{
    while(true)
    {
        //Create lock and wait until notified (with new task) or continue directly if the job queue is not empty
        //The mutex is locked after wait returned
        //About the wait condition: Only locks if false (else continues right away), only releases if true
        std::unique_lock<std::mutex> lock(job_mutex);
        jobs_available.wait(lock, [this] { return stop_threads.load() || !job_queue.empty(); });

        //If threads should no longer be running, stop the execution of the worker thread
        if (stop_threads.load())
        {
            return;
        }

        //Get the next task
        auto msg = job_queue.front();
        job_queue.pop();

        //Unlock the mutex
        lock.unlock();

        //Do the task
        process_single_child_command(msg);
    }
}

void ProgramExecutor::stop_thread_pool()
{
    //First acquire a lock, s.t. waiting threads cannot acquire it during setting stop_threads
    std::unique_lock<std::mutex> lock(job_mutex);
    stop_threads.store(true);
    lock.unlock();

    //All threads that are woken up now stop immediately due to the implementation in do_jobs_or_stop
    //Other threads should watch for changes in stop_threads
    jobs_available.notify_all();

    //Join all threads
    for(auto& t : thread_pool)
    {
        if (t.joinable()) t.join();
    }
}

void ProgramExecutor::process_single_child_command(CommandMsg& msg)
{
    std::string msg_string = msg.command.command;

    //Execute the command based on the command type / timeout
    if (msg.command.request_type == RequestType::SEND_OUTPUT)
    {
        //Sometimes, this type of command seems to hang up, for unknown reasons. Thus, a timeout is added in this case.
        //SIGTERM is set after 10 seconds, SIGKILL after 12 if TERM was not enough
        //After the timeout, if the process was killed due to a timeout, ERROR is returned
        std::stringstream stream;
        stream << "timeout -k 12 10 " << msg.command.command << "; if [ $? != 0 ]; then echo 'ERROR'; fi";
        std::string output = execute_command_get_output(stream.str().c_str());
        
        //Create and send answer, repeat in case of failure (as the main process waits for it)
        while(! send_answer_msg(msg_response_queue_id, output, msg.mtype, true))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    else if (msg.command.timeout_seconds > 0)
    {
        bool exec_success = spawn_and_manage_process(msg_string.c_str(), msg.command.timeout_seconds);

        //Create and send answer, repeat in case of failure (as the main process waits for it)
        while(! send_answer_msg(msg_response_queue_id, "", msg.mtype, exec_success))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    else
    {
        //We just want to execute the command without timeouts
        //Simply use system() in this case
        system(msg_string.c_str());
    }
}

bool ProgramExecutor::execute_command(std::string command, int timeout)
{
    auto command_id = get_unique_command_id();

    //Send a msg to the child process, telling it to execute the given command
    CommandMsg msg;
    if (create_command_msg(command, msg, command_id, timeout))
    {
        bool send_success = send_command_msg(msg_request_queue_id, msg);

        //If the msg could not be sent, return false
        if (!send_success) return false;
    }
    else
    {
        //Do not use log() here - is not a child function. Plus: The caller should ideally react to "false" with its own log.
        std::cerr << "ERROR: Could not create IPC Message, command string was too large" << std::endl;
        return false;
    }

    //Wait for an answer regarding the process state of the command in case a timeout was set
    if (timeout > 0)
    {
        AnswerMsg response;
        if (receive_answer_msg(msg_response_queue_id, command_id, response))
        {
            return response.answer.execution_success;
        }
        else return false;
    }
    else return true;
}

std::string ProgramExecutor::get_command_output(std::string command)
{
    auto command_id = get_unique_command_id();

    //Send a msg to the child process, telling it to execute the given command
    CommandMsg msg;
    if (create_command_msg(command, msg, command_id, -1, RequestType::SEND_OUTPUT))
    {
        send_command_msg(msg_request_queue_id, msg);
    }
    else
    {
        //Do not use log() here - is not a child function. Plus: The caller should ideally react to "false" with its own log.
        std::cerr << "ERROR: Could not create IPC Message, command string was too large" << std::endl;
        return "ERROR";
    }

    //Now wait for the answer / received command output
    AnswerMsg response;
    if (receive_answer_msg(msg_response_queue_id, command_id, response))
    {
        std::string string_response(response.answer.truncated_command_output);

        if (string_response.find("ERROR") != std::string::npos)
        {
            //Also do not use log here, ERROR is already returned so we do not handle this further
            std::cerr << "ERROR while waiting for response!" << std::endl;
        }

        return response.answer.truncated_command_output;
    }
    else {
        return "ERROR";
    }
}

int ProgramExecutor::create_msg_queue(std::string filepath)
{
    //To get a (hopefully) unqiue ID, we need to create one given a file location
    //For better portability, we hope that argv[0] contains our current file location and use that 
    //(It usually should)#
    //NOTE: We do not use further control or error checking (e.g. in case of a full queue) right now,
    //because due to the desired way of operation we do not expect the queue to become full

    //Uses the log function because it is used before child creation

    auto key = ftok(filepath.c_str(), 'a'); //The char is usually arbitrary, I chose a

    if (key == -1)
    {
        log("ERROR: Could not create IPC Key (ftok) which is required for communicating commands to start external programs!");
        std::cerr << "THE LCC WILL NOW SHUT DOWN" << std::endl;
        throw std::runtime_error("ERROR: Could not create IPC Key (ftok) which is required for communicating commands to start external programs!");
    }

    int queue_id = msgget(key, IPC_CREAT | 0666); //Permissions: rw-rw-rw-

    if (queue_id == -1)
    {
        log("ERROR: Could not create IPC Message Queue (msgget) which is required for communicating commands to start external programs!");
        std::cerr << "THE LCC WILL NOW SHUT DOWN" << std::endl;
        throw std::runtime_error("ERROR: Could not create IPC Message Queue (msgget) which is required for communicating commands to start external programs!");
    }

    return queue_id;
}

void ProgramExecutor::destroy_msg_queue(int msg_queue_id)
{
    if (msg_queue_id >= 0)
    {
        auto return_code = msgctl(msg_queue_id, IPC_RMID, NULL);
        if (return_code == -1)
        {
            std::cerr << "ERROR: Could not kill IPC Message Queue: " << std::strerror(errno) << std::endl;
        }
    }
}


bool ProgramExecutor::create_command_msg(
    std::string command_string, 
    CommandMsg& command_out, 
    long command_id, 
    int timeout_seconds, 
    RequestType request_type
)
{
    command_out.command.timeout_seconds = timeout_seconds;
    command_out.command.request_type = request_type;
    command_out.mtype = command_id; // To be able to later on identify the response with the sent request

    //WARNING: In a real-world scenario, do not forget to make sure that the command string is shorter than the size of
    //command_out.command.command, else we do not get a null terminated C string or need to truncate (which would be undesirable as well)
    std::strncpy(command_out.command.command, command_string.c_str(), sizeof(command_out.command.command));
    command_out.command.command[sizeof(command_out.command.command)-1] = '\0'; //For safety reasons, in case the string is too long

    //Now: Compare the copied string to the original, return false if this failed
    if (std::strcmp(command_out.command.command, command_string.c_str()) != 0) 
        return false;

    return true;
}

bool ProgramExecutor::send_command_msg(int msqid, CommandMsg& msg)
{
    //Send the message. There might be some problem with it, so do not forget to check for errors. The flag is not used (0)
    auto return_code = msgsnd(msqid, &msg, sizeof(CommandMsg::CommandInfo), 0);
    if (return_code == -1)
    {
        //Not called by child, so do not log this further
        std::cerr << "ERROR: Could not send IPC Message: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool ProgramExecutor::receive_command_msg(int msqid, CommandMsg& msg)
{
    //Set msgtyp here to 0, because we want to receive any command msg (ID is not relevant for child, only for sending answers)
    auto return_code = msgrcv(msqid, &msg, sizeof(CommandMsg::CommandInfo), 0, 0);
    if (return_code == -1)
    {
        //Use log, as only the child receives command messages
        std::stringstream error_msg;
        error_msg << "ERROR: Could not receive IPC Message: " << std::strerror(errno);
        log(error_msg.str());
        return false;
    }

    return true;
}

std::string ProgramExecutor::execute_command_get_output(const char* cmd)
{
    //Code from stackoverflow
    // std::array<char, 128> buffer;
    // std::string result;
    // std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    // if (!pipe) {
    //     throw std::runtime_error("Could not use popen - deployment failed!");
    // }
    // while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    //     result += buffer.data();
    // }
    // return result;

    //We want to be able to kill the process early, so popen() is not really an option
    //Do smth similar to spawn_and_manage_process instead
    int command_pipe[2];

    if (pipe(command_pipe))
    {
        std::stringstream error_msg;
        error_msg << "Pipe creation failed in execute_command_get_output, command: " << cmd;
        log(error_msg.str());
        return "ERROR";
    }

    int process_id = fork();
    if (process_id == 0)
    {
        //Tell the child to set its group process ID to its process ID, or else things like kill(-pid) to kill a ping-while-loop won't work
        setpgid(0, 0);

        //We only want to write stdout, close the pipe's listen end
        dup2(command_pipe[1], STDOUT_FILENO);
        close(command_pipe[0]);
        
        //Actions to take within the new child process
        //ACHTUNG: NUTZE chmod u+x für die Files, sonst: permission denied
        execl("/bin/sh", "bash", "-c", cmd, NULL);

        //Error if execlp returns, cannot log this as this is not within the "parent" child
        std::cerr << "Execl error in ProgramExecutor class: %s, for execution of '%s'" << std::strerror(errno) << cmd << std::endl;

        //Write to pipe because some response is still required / waited for
        write(command_pipe[1], "ERROR", 5);
        close(command_pipe[1]);

        exit(EXIT_FAILURE);
    }
    else if (process_id > 0)
    {
        //We are in the parent process and got the child's PID

        //We only want to listen, close the pipe's write end
        close(command_pipe[1]);

        //Get the output until no more characters are available
        //!!!!!!!!!!!!!!!!!!!!!!!!
        //TODO / NOTE: This might be very inefficient (mutex lock for each character for stop_threads.load()), shall I improve this?
        //!!!!!!!!!!!!!!!!!!!!!!!!
        char ch;
        std::string out;
        while (read(command_pipe[0], &ch, 1) && !stop_threads.load())
        {
            out += ch;
        }

        //Finally, close the pipe and kill the process
        if (get_child_process_state(process_id) == PROCESS_STATE::DONE)
        {
            //std::cout << "Success: execution of " << cmd << std::endl;

            //Clean up by waiting / telling the child that it can "destroy itself"
            int status;
            waitpid(process_id, &status, 0); //0 -> no flags here
        }
        else
        {
            kill_process(process_id, 100);
        }

        close(command_pipe[0]);

        //Return the obtained msg
        return out;
    }
    else 
    {
        //We could not spawn a new process - usually, the program should not just break at this point, unless that behaviour is desired
        std::stringstream error_msg;
        error_msg << "Error in ProgramExecutor class: Could not create child process in execute_command_get_output, command: " << cmd;
        log(error_msg.str());
        return "ERROR";
    }
}

bool ProgramExecutor::send_answer_msg(int msqid, std::string command_output, long command_id, bool execution_success)
{
    AnswerMsg answer_msg;

    answer_msg.answer.execution_success = execution_success;
    answer_msg.mtype = command_id; //To identify sent response with request

    //WARNING: In a real-world scenario, do not forget to make sure that the command string is shorter than the size of
    //command_out.command.command, else we do not get a null terminated C string or need to truncate (which would be undesirable as well)
    std::strncpy(answer_msg.answer.truncated_command_output, command_output.c_str(), sizeof(answer_msg.answer.truncated_command_output));
    answer_msg.answer.truncated_command_output[sizeof(answer_msg.answer.truncated_command_output)-1] = '\0'; //For safety reasons, in case the string is too long

    //Now: Compare the copied string to the original, print warning if the output had to be truncated
    if (std::strcmp(answer_msg.answer.truncated_command_output, command_output.c_str()) != 0) 
    {
        //Only called by child, so we can use log
        std::stringstream error_msg;
        error_msg << "WARNING: The requested command output was too large and had to be truncated. This may lead to undesired program behavior! Output: " << command_output;
        log(error_msg.str());
    }

    //Send the message. There might be some problem with it, so do not forget to check for errors. The flag is not used (0)
    auto return_code = msgsnd(msqid, &answer_msg, sizeof(AnswerMsg::AnswerInfo), 0);
    if (return_code == -1)
    {
        //Only called by child, so we can use log
        std::stringstream error_msg;
        error_msg << "ERROR: Could not send IPC Message: " << std::strerror(errno) << ", msg was: " << command_output;
        log(error_msg.str());
        return false;
    }

    return true;
}

bool ProgramExecutor::receive_answer_msg(int msqid, long command_id, AnswerMsg& msg)
{
    auto return_code = msgrcv(msqid, &msg, sizeof(AnswerMsg::AnswerInfo), command_id, 0);
    if (return_code == -1)
    {
        //Cannot use log function here, as this is a parent error msg
        std::cerr << "ERROR: Could not receive IPC Message: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}


int ProgramExecutor::execute_command_get_pid(const char* cmd)
{
    int process_id = fork();
    if (process_id == 0)
    {
        //Tell the child to set its group process ID to its process ID, or else things like kill(-pid) to kill a ping-while-loop won't work
        setpgid(0, 0);
        
        //Actions to take within the new child process
        //ACHTUNG: NUTZE chmod u+x für die Files, sonst: permission denied
        execl("/bin/sh", "bash", "-c", cmd, NULL);

        //Error if execlp returns, cannot log this as this is not the "master" child
        std::cerr << "Execl error in ProgramExecutor class: %s, for execution of '%s'" << std::strerror(errno) << cmd << std::endl;

        exit(EXIT_FAILURE);
    }
    else if (process_id > 0)
    {
        //We are in the parent process and got the child's PID
        return process_id;
    }
    else 
    {
        //We could not spawn a new process - usually, the program should not just break at this point, unless that behaviour is desired
        std::stringstream error_msg;
        error_msg << "Error in execute_command_get_pid: Could not create child process for " << cmd;
        log(error_msg.str());
        exit(EXIT_FAILURE);
    }
}

ProgramExecutor::PROCESS_STATE ProgramExecutor::get_child_process_state(int process_id)
{
    int process_status;
    pid_t result = waitpid(process_id, &process_status, WNOHANG);

    if (result == 0)
    {
        return PROCESS_STATE::RUNNING;
    }
    else if (result == -1)
    {
        return PROCESS_STATE::ERROR;
    }
    else
    {
        return PROCESS_STATE::DONE;
    }
    
}

void ProgramExecutor::kill_process(int process_id, int timeout_ms)
{
    //Tell the process to terminate - this way, it can terminate gracefully
    //We mostly use bash, were whole process groups might be created - to kill those, we need the negative id
    if (process_id > 0)
    {
        process_id *= (-1);
    }

    kill(process_id, SIGTERM);

    //Wait for the process to terminate
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));

    //Check if the process terminated as desired
    PROCESS_STATE state = get_child_process_state(process_id);

    //If the process has not yet terminated, force a termination and clean up
    if (state != PROCESS_STATE::DONE)
    {
        kill(process_id, SIGKILL);
        int status;
        waitpid(process_id, &status, 0); //0 -> no flags here
    }
}

bool ProgramExecutor::spawn_and_manage_process(const char* cmd, unsigned int timeout_seconds)
{
    //std::cout << "Executing " << cmd << std::endl;

    //Spawn and manage new process
    int process_id = execute_command_get_pid(cmd);
    auto start_time = std::chrono::high_resolution_clock::now();

    auto time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //Regularly check status during execution until timeout - exit early if everything worked as planned, else run until error / timeout and return error
    while (time_passed_ms < static_cast<int64_t>(timeout_seconds) * 1000)
    {
        //Check current program state
        PROCESS_STATE state = get_child_process_state(process_id);

        if (state == PROCESS_STATE::DONE)
        {
            //std::cout << "Success: execution of " << cmd << std::endl;

            //Clean up by waiting / telling the child that it can "destroy itself"
            int status;
            waitpid(process_id, &status, 0); //0 -> no flags here
            return true;
        }
        else if (state == PROCESS_STATE::ERROR || stop_threads.load())
        {
            kill_process(process_id, 100);
            //std::cout << "Error in execution of " << cmd << std::endl;
            return false;
        }

        //Use longer sleep time until short before end of timeout
        time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        auto remaining_time = static_cast<int64_t>(timeout_seconds) * 1000 - time_passed_ms;
        if (remaining_time > 500)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else if (remaining_time > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(remaining_time));
        }
        
        time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    }

    //Now kill the process, as it has not yet finished its execution
    //std::cout << "Killing" << std::endl;
    kill_process(process_id, 100);
    //std::cout << "Could not execute in time: " << cmd << std::endl;
    return false;
}

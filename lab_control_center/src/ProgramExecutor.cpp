#include "ProgramExecutor.hpp"

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
    //Create msg queues (must be done before forking, so that both processes have a working reference)
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

        exit(EXIT_SUCCESS);
    }
    else if (child_process_id > 0)
    {
        //We are in the parent process and can now proceed in main
        return true;
    }
    else 
    {
        //We could not spawn a new process - usually, the program should not just break at this point, unless that behaviour is desired
        //TODO: Change behaviour
        std::cerr << "Error when using fork: Could not create 'meta'-child process!";
        return false;
    }
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
        std::string output = execute_command_get_output(msg_string.c_str());
        
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
        std::cerr << "ERROR: Could not create IPC Message, command string was too large" << std::endl;
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
        std::cerr << "ERROR: Could not create IPC Message, command string was too large" << std::endl;
    }

    //Now wait for the answer / received command output
    AnswerMsg response;
    if (receive_answer_msg(msg_response_queue_id, command_id, response))
    {
        return response.answer.truncated_command_output;
    }
    else return "ERROR";
}

int ProgramExecutor::create_msg_queue(std::string filepath)
{
    //To get a (hopefully) unqiue ID, we need to create one given a file location
    //For better portability, we hope that argv[0] contains our current file location and use that 
    //(It usually should)#
    //NOTE: We do not use further control or error checking (e.g. in case of a full queue) right now,
    //because due to the desired way of operation we do not expect the queue to become full
    auto key = ftok(filepath.c_str(), 'a'); //The char is usually arbitrary, I chose a

    if (key == -1)
    {
        std::cerr << "ERROR: Could not create IPC Key (ftok) which is required for communicating commands to start external programs!" << std::endl;
        exit(EXIT_FAILURE);
    }

    int queue_id = msgget(key, IPC_CREAT | 0666); //Permissions: rw-rw-rw-

    if (queue_id == -1)
    {
        std::cerr << "ERROR: Could not create IPC Message Queue (msgget) which is required for communicating commands to start external programs!" << std::endl;
        exit(EXIT_FAILURE);
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
        std::cerr << "ERROR: Could not receive IPC Message: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

std::string ProgramExecutor::execute_command_get_output(const char* cmd)
{
    //Code from stackoverflow
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("Could not use popen - deployment failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
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
        std::cerr << "\n!!!\n!!!\nWARNING: The requested command output was too large and had to be truncated. This may lead to undesired program behavior!\n!!!\n!!!\n" << std::endl;

    //Send the message. There might be some problem with it, so do not forget to check for errors. The flag is not used (0)
    auto return_code = msgsnd(msqid, &answer_msg, sizeof(AnswerMsg::AnswerInfo), 0);
    if (return_code == -1)
    {
        std::cerr << "ERROR: Could not send IPC Message: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool ProgramExecutor::receive_answer_msg(int msqid, long command_id, AnswerMsg& msg)
{
    auto return_code = msgrcv(msqid, &msg, sizeof(AnswerMsg::AnswerInfo), command_id, 0);
    if (return_code == -1)
    {
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

        //Error if execlp returns
        std::cerr << "Execl error in Deploy class: %s, for execution of '%s'" << std::strerror(errno) << cmd << std::endl;

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
        //TODO: Change behaviour
        std::cerr << "Error in Deploy class: Could not create child process for " << cmd << std::endl;
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
            kill_process(process_id);
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
    kill_process(process_id);
    //std::cout << "Could not execute in time: " << cmd << std::endl;
    return false;
}

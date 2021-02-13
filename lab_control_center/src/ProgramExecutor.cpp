#include "ProgramExecutor.hpp"

ProgramExecutor::~ProgramExecutor()
{
    //Only do something if not the child
    if (child_process_id != 0)
    {
        //Send final msg to tell the child to stop its execution
        //Only do this if child process creation was successful
        if (child_process_id > 0)
        {
            CommandMsg msg;
            if (create_command_msg("EXIT", msg))
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

        //Now try to receive a message; the child waits if there currently is no message present
        //0 to receive the next message on the queue, irrelevant of the value of mtype, flag not used (0 as well)
        //TODO: Create a test scenario where the child gets killed while waiting, to test if that works
        while (true)
        {
            CommandMsg msg;
            receive_command_msg(msg_request_queue_id, msg);

            //Exit condition - just compare the first characters, as strncpy fills up the char array
            std::string msg_string = msg.command.command;
            if (msg_string.compare(0, 4, "EXIT") == 0)
                break;

            //Debug log (can't use logger here, or any other cpm library function, because some parts of it use DDS)
            std::cout << "Child received command: " << msg_string << std::endl;

            //Now execute the command based on whether the timeout was set
            if (msg.command.request_type == RequestType::SEND_OUTPUT)
            {
                std::string output = execute_command_get_output(msg_string.c_str());
                
                //Create and send answer
                send_answer_msg(msg_response_queue_id, output, true);
            }
            else if (msg.command.timeout_seconds > 0)
            {
                bool exec_success = spawn_and_manage_process(msg_string.c_str(), msg.command.timeout_seconds);

                //Create and send answer
                send_answer_msg(msg_response_queue_id, "", exec_success);
            }
            else
            {
                //We just want to execute the command without timeouts
                //Simply use system() in this case
                system(msg_string.c_str());
            }
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

bool ProgramExecutor::execute_command(std::string command, int timeout)
{
    //Send a msg to the child process, telling it to execute the given command
    CommandMsg msg;
    if (create_command_msg(command, msg, timeout))
    {
        send_command_msg(msg_request_queue_id, msg);
    }
    else
    {
        std::cerr << "ERROR: Could not create IPC Message, command string was too large" << std::endl;
    }

    //Wait for an answer regarding the process state of the command in case a timeout was set
    if (timeout > 0)
    {
        AnswerMsg response;
        receive_answer_msg(msg_response_queue_id, response);
        return response.answer.execution_success;
    }
    else return true;
}

std::string ProgramExecutor::get_command_output(std::string command)
{
    //Send a msg to the child process, telling it to execute the given command
    CommandMsg msg;
    if (create_command_msg(command, msg, -1, RequestType::SEND_OUTPUT))
    {
        send_command_msg(msg_request_queue_id, msg);
    }
    else
    {
        std::cerr << "ERROR: Could not create IPC Message, command string was too large" << std::endl;
    }

    //Now wait for the answer / received command output
    AnswerMsg response;
    receive_answer_msg(msg_response_queue_id, response);
    return response.answer.truncated_command_output;
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


bool ProgramExecutor::create_command_msg(std::string command_string, CommandMsg& command_out, int timeout_seconds, RequestType request_type)
{
    command_out.command.timeout_seconds = timeout_seconds;
    command_out.command.request_type = request_type;
    command_out.mtype = 1; //Irrelevant for us

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

bool ProgramExecutor::send_answer_msg(int msqid, std::string command_output, bool execution_success)
{
    AnswerMsg answer_msg;

    answer_msg.answer.execution_success = execution_success;
    answer_msg.mtype = 1; //Irrelevant for us

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

bool ProgramExecutor::receive_answer_msg(int msqid, AnswerMsg& msg)
{
    auto return_code = msgrcv(msqid, &msg, sizeof(AnswerMsg::AnswerInfo), 0, 0);
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
        else if (state == PROCESS_STATE::ERROR)
        {
            kill_process(process_id);
            //std::cout << "Error in execution of " << cmd << std::endl;
            return false;
        }

        //Use longer sleep time until short before end of timeout
        time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        auto remaining_time = static_cast<int64_t>(timeout_seconds) * 1000 - time_passed_ms;
        if (remaining_time > 1000)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
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

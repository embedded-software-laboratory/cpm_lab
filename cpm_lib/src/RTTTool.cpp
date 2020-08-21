#include "cpm/RTTTool.hpp"

cpm::RTTTool::RTTTool() : 
    rtt_topic(cpm::get_topic<RoundTripTime>(cpm::ParticipantSingleton::Instance(), "round_trip_time")),
    rtt_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), rtt_topic)
{
    rtt_measure_requested.store(false);
    rtt_count.store(0);

    //Create reader that directly answers the messages and stores times of RTT answers
    rtt_reader = std::make_shared<cpm::AsyncReader<RoundTripTime>>(
        [this](dds::sub::LoanedSamples<RoundTripTime>& samples){
            for(auto sample : samples)
            {
                if(sample.info().valid())
                {
                    //Only answer to the RTT request if the program ID is different from the own program ID
                    if (!(sample.data().is_answer()) && sample.data().source_id() != program_id)
                    {
                        RoundTripTime answer;
                        answer.count(sample.data().count());
                        answer.is_answer(true);
                        answer.source_id(program_id);
                        rtt_writer.write(answer);
                    }
                    else if (sample.data().is_answer() && rtt_measure_requested.load() && sample.data().count() == rtt_count.load())
                    {
                        //Store RTT answers if they were requested and match the current request count
                        std::lock_guard<std::mutex> lock(receive_times_mutex);
                        receive_times.push_back(cpm::get_time_ns());
                    }
                }
            }
        },
        cpm::ParticipantSingleton::Instance(),
        rtt_topic,
        false,
        false
    );
}

cpm::RTTTool& cpm::RTTTool::Instance()
{
    static RTTTool instance;
    return instance;
}

void cpm::RTTTool::set_id(std::string _program_id)
{
    program_id = _program_id;
}


std::pair<uint64_t, uint64_t> cpm::RTTTool::measure_rtt()
{
    //First, increment the RTT count, so that old messages are discarded properly
    auto current_count = rtt_count.load();
    if (current_count < 255)
    {
        ++current_count;
    }
    else
    {
        current_count = 0;
    }
    rtt_count.store(current_count);
    

    //Then, clear the vector containing old answers (and hope that no old messages are still being sent)
    std::unique_lock<std::mutex> lock(receive_times_mutex);
    receive_times.clear();
    lock.unlock();

    RoundTripTime request;
    request.is_answer(false);
    request.source_id(program_id);
    request.count(current_count);

    rtt_measure_requested.store(true);
    auto start_time = cpm::get_time_ns();
    rtt_writer.write(request);

    //Wait for answers to appear in the RTT times vector
    unsigned int wait_count = 0; //Do not wait longer than 5 seconds
    lock.lock();
    while (receive_times.size() == 0)
    {
        lock.unlock();

        if (wait_count == 5)
        {
            break;
        }
        ++ wait_count;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        lock.lock();
    }

    //Check if the 5 second timeout was reached before any message was received
    if (wait_count < 5)
    {
        //Now that we got answers, calculate the 'fastest' RTT - due to the saving order, this is stored in the first entry of the vector
        auto fastest_answer = receive_times.at(0);
        lock.unlock();

        //Also wait another 200ms to get a worse RTT time
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        lock.lock();
        auto amnt_answers = receive_times.size();
        auto slowest_answer = receive_times.at(amnt_answers - 1); //Could be the same as the fastest one
        lock.unlock();

        //Now calculate 'best' and 'worst' RTT
        auto best_rtt = fastest_answer - start_time;
        auto worst_rtt = slowest_answer - start_time;

        //Currently, we do not consider who the best / worst RTT belongs to
        return {best_rtt, worst_rtt};
    }
    else
    {
        return {0, 0};
    }
}
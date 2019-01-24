#include "TimerFD.hpp"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <sys/timerfd.h>
#include <unistd.h>

TimerFD::TimerFD(
    std::string _node_id, 
    uint64_t _period_nanoseconds, 
    uint64_t _offset_nanoseconds
)
:period_nanoseconds(_period_nanoseconds)
,offset_nanoseconds(_offset_nanoseconds)
,node_id(_node_id)
{
    // Timer setup
    timer_fd = timerfd_create(CLOCK_REALTIME, 0);
    if (timer_fd == -1) {
        fprintf(stderr, "Call to timerfd_create failed.\n"); 
        perror("timerfd_create");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

    uint64_t offset_nanoseconds_fd = offset_nanoseconds;

    if(offset_nanoseconds_fd == 0) { // A zero value disarms the timer, overwrite with a negligible 1 ns.
        offset_nanoseconds_fd = 1;
    }

    struct itimerspec its;
    its.it_value.tv_sec     = offset_nanoseconds_fd / 1000000000ull;
    its.it_value.tv_nsec    = offset_nanoseconds_fd % 1000000000ull;
    its.it_interval.tv_sec  = period_nanoseconds / 1000000000ull;
    its.it_interval.tv_nsec = period_nanoseconds % 1000000000ull;
    int status = timerfd_settime(timer_fd, TFD_TIMER_ABSTIME, &its, NULL);
    if (status != 0) {
        fprintf(stderr, "Call to timer_settime returned error status (%d).\n", status);
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
}

void TimerFD::wait()
{
    unsigned long long missed;
    int status = read(timer_fd, &missed, sizeof(missed));
    if(status != sizeof(missed)) {
        fprintf(stderr, "Error: read(timerfd), status %d.\n", status);
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
}

void TimerFD::waitForStart() {
    // Timer setup
    int timer = timerfd_create(CLOCK_REALTIME, 0);
    if (timer == -1) {
        fprintf(stderr, "Call to timerfd_create in waitForStart failed.\n"); 
        perror("timerfd_create");
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

    //Reader / Writer for ready status and system trigger
    dds::topic::Topic<ReadyStatus> ready_topic(cpm::ParticipantSingleton::Instance(), "ready");
    dds::pub::DataWriter<ReadyStatus> writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), ready_topic);
    dds::topic::Topic<SystemTrigger> trigger_topic(cpm::ParticipantSingleton::Instance(), "system_trigger");
    dds::sub::DataReader<SystemTrigger> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic);

    //Waitset to wait for data
    // Create a WaitSet
    dds::core::cond::WaitSet waitset;
    // Create a GuardCondition
    dds::core::cond::GuardCondition guard_cond;
    // Create a StatusCondition for a given Entity
    dds::core::cond::StatusCondition status_cond(reader);
    status_cond.enabled_statuses(dds::core::status::StatusMask::data_available());
    // Create a ReadCondition for a reader with a specific DataState
    dds::sub::cond::ReadCondition read_cond(
        reader, dds::sub::status::DataState(
            dds::sub::status::SampleState::not_read(),
            dds::sub::status::ViewState::any(),
            dds::sub::status::InstanceState::any()));
    // Attach conditions
    waitset += guard_cond; // using += operator
    waitset += status_cond;
    waitset.attach_condition(read_cond); // or using attach_condition()


    //Send ready signal
    ReadyStatus ready_status;
    ready_status.next_start_stamp(TimeStamp(0));
    ready_status.source_id(node_id);
    writer.write(ready_status);

    std::cout << "Signal sent" << std::endl;
    
    //Wait for start signal
    SystemTrigger trigger;
    dds::core::cond::WaitSet::ConditionSeq active_conditions =
        waitset.wait();
    reader.take(trigger);

    std::cout << "Got system trigger " << trigger.next_start().nanoseconds() << std::endl;

    //Finish timer setup
    struct itimerspec its;
    its.it_value.tv_sec     = 0;
    its.it_value.tv_nsec    = trigger.next_start().nanoseconds() % 1000000000ull;
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = 0;
    int status = timerfd_settime(timer, TFD_TIMER_ABSTIME, &its, NULL);
    if (status != 0) {
        fprintf(stderr, "Call to timer_settime returned error status (%d).\n", status);
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

    std::cout << "Waiting for starting point" << std::endl;

    //Wait for starting point
    unsigned long long missed;
    status = read(timer, &missed, sizeof(missed));
    if(status != sizeof(missed)) {
        fprintf(stderr, "Error: read(timer), status %d.\n", status);
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }
}

void TimerFD::start(std::function<void(uint64_t t_now)> update_callback)
{
    if(this->active) {
        std::cerr << "The cpm::Timer can not be started twice" << std::endl;
        return;
    }

    m_update_callback = update_callback;
    
    uint64_t deadline = ((this->get_time()/period_nanoseconds)+1)*period_nanoseconds + offset_nanoseconds;
    this->active = true;

    //Send ready signal, wait for start signal
    waitForStart();

    while(this->active) {
        this->wait();
        if(this->get_time() >= deadline) {
            if(m_update_callback) m_update_callback(deadline);

            deadline += period_nanoseconds;

            while(this->get_time() >= deadline)
            {
                std::cerr << "Warning, missed timestep " << deadline << std::endl;
                deadline += period_nanoseconds;
            }
        }
    }
}

void TimerFD::start_async(std::function<void(uint64_t t_now)> update_callback)
{
    if(!runner_thread.joinable())
    {
        m_update_callback = update_callback;
        runner_thread = std::thread([this](){
            this->start(m_update_callback);
        });
    }
    else
    {
        std::cerr << "The cpm::Timer can not be started twice" << std::endl;
    }
}

void TimerFD::stop()
{
    active = false;
    if(runner_thread.joinable())
    {
        runner_thread.join();
    }
}

TimerFD::~TimerFD()
{
    active = false;
    if(runner_thread.joinable())
    {
        runner_thread.join();
    }
}


uint64_t TimerFD::get_time()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}
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
,ready_topic(cpm::ParticipantSingleton::Instance(), "ready")
,trigger_topic(cpm::ParticipantSingleton::Instance(), "system_trigger")
,node_id(_node_id)
{
    //Offset must be smaller than period
    if (_offset_nanoseconds >= _period_nanoseconds) {
        _offset_nanoseconds = _period_nanoseconds - 1;
    }

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
    dds::pub::DataWriter<ReadyStatus> writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), ready_topic, (dds::pub::qos::DataWriterQos() << dds::core::policy::Reliability::Reliable()));
    dds::sub::DataReader<SystemTrigger> reader(dds::sub::Subscriber(cpm::ParticipantSingleton::Instance()), trigger_topic, (dds::sub::qos::DataReaderQos() << dds::core::policy::Reliability::Reliable()));

    //Create ready signal
    ReadyStatus ready_status;
    ready_status.next_start_stamp(TimeStamp(0));
    ready_status.source_id(node_id);
    
    //Poll for start signal, send ready signal every 2 seconds until the start signal has been received
    bool noSignalReceived = true;
    SystemTrigger trigger;
    do {
        writer.write(ready_status);

        rti::util::sleep(dds::core::Duration(2));

        for (auto sample : reader.take()) {
            if (sample.info().valid()) {
                trigger.next_start(sample.data().next_start());
                noSignalReceived = false;
                break;
            }
        }
    }
    while(noSignalReceived);

    //Finish timer setup
    struct itimerspec its;
    its.it_value.tv_sec     = trigger.next_start().nanoseconds() / 1000000000ull;
    its.it_value.tv_nsec    = trigger.next_start().nanoseconds() % 1000000000ull;
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = 0;
    int status = timerfd_settime(timer, TFD_TIMER_ABSTIME, &its, NULL);
    if (status != 0) {
        fprintf(stderr, "Call to timer_settime returned error status (%d).\n", status);
        fflush(stderr); 
        exit(EXIT_FAILURE);
    }

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
    else if (timer_fd == -1) {
        std::cerr << "The cpm::Timer has been deleted, please create a new object" << std::endl;
        return;
    }

    m_update_callback = update_callback;

    //Send ready signal, wait for start signal
    waitForStart();
    
    uint64_t deadline = ((this->get_time()/period_nanoseconds)+1)*period_nanoseconds + offset_nanoseconds;
    this->active = true;

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
    close(timer_fd);
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
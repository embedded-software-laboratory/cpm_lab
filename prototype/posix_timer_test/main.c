#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>

#define SIG SIGRTMAX

#define errExit(msg)    do { perror(msg); exit(EXIT_FAILURE); \
                        } while (0)


static void handler(int sig, siginfo_t *si, void *uc){}

int main(int argc, char *argv[])
{
    /* Establish handler for timer signal */
    struct sigaction signal_action;
    signal_action.sa_flags = SA_SIGINFO;
    signal_action.sa_sigaction = handler;
    sigemptyset(&signal_action.sa_mask);
    if (sigaction(SIG, &signal_action, NULL) == -1)
        errExit("sigaction");

    /* Create the timer */
    timer_t timerid;
    struct sigevent signal_event;
    signal_event.sigev_notify = SIGEV_SIGNAL;
    signal_event.sigev_signo = SIG;
    signal_event.sigev_value.sival_ptr = &timerid;
    if (timer_create(CLOCK_REALTIME, &signal_event, &timerid) == -1)
        errExit("timer_create");

    /* Start the timer */
    struct itimerspec timer_spec;
    timer_spec.it_value.tv_sec = 0;
    timer_spec.it_value.tv_nsec = 500000000L;
    timer_spec.it_interval.tv_sec = 1;
    timer_spec.it_interval.tv_nsec = 0;

    if (timer_settime(timerid, TIMER_ABSTIME , &timer_spec, NULL) == -1)
         errExit("timer_settime");


    sigset_t signal_mask;
    sigfillset(&signal_mask);
    sigdelset(&signal_mask, SIG);
    while(1) {
        sigsuspend(&signal_mask);
        system("date +%s.%N");
    }

    exit(EXIT_SUCCESS);
}
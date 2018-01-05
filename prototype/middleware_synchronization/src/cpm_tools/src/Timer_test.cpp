

#include "Timer.h"
#include "cpm_tools/default.h"


/*
http://man7.org/linux/man-pages/man2/eventfd.2.html
http://www.2net.co.uk/tutorial/periodic_threads
https://stackoverflow.com/questions/26304673/repeating-timerfd-event-works-with-epoll-and-not-with-poll
https://www.ulduzsoft.com/2014/01/select-poll-epoll-practical-difference-for-system-architects/
http://man7.org/linux/man-pages/man2/timerfd_create.2.html

*/

int main() {
    auto dt = ros::Duration(2,0);
    Timer t(dt);
    cout << "helo" << endl;
}

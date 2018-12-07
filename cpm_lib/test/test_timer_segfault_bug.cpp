
#include "TimerFD.hpp"
#include <unistd.h>


class dummy_class
{
    std::shared_ptr<cpm::Timer> update_loop = nullptr;

public:
    dummy_class()
    {
        update_loop = cpm::Timer::create("sdfgsdfg", 20000000ull, 0);
        update_loop->start_async([](uint64_t){});
    }
};


int main() {

    dummy_class sdfgsdfg;
    // this causes a SIGSEGV, WTF?
    sleep(100);

}
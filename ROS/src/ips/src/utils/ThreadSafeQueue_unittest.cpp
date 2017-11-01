#include "tools/unittest/catch.hpp"
#include "ThreadSafeQueue.h"
#include <thread>
#include <chrono>


TEST_CASE("ThreadSafeQueue_PreservesDataOrder") {

    ThreadSafeQueue<int, 17> Q;

    for (int j = 0; j < 11; ++j) {
        for (int i = 5; i < 45; i += 3) {
            Q.write(i);
        }
        for (int i = 5; i < 45; i += 3) {
            int d;
            CHECK( Q.read(d) == true );
            CHECK( d == i );
        }
    }
}


TEST_CASE("ThreadSafeQueue_CloseWhileReading") {

    using namespace std::chrono_literals;

    ThreadSafeQueue<int, 1> Q;

    std::thread t([&](){
        std::this_thread::sleep_for(5ms);
        Q.close();
    });
    
    int d;
    CHECK( Q.write(1) == true );
    CHECK( Q.read(d) == true );
    CHECK( Q.read(d) == false );

    t.join();
}


TEST_CASE("ThreadSafeQueue_CloseWhileWriting") {

    using namespace std::chrono_literals;

    ThreadSafeQueue<int, 2> Q;

    std::thread t([&](){
        std::this_thread::sleep_for(5ms);
        Q.close();
    });
    
    CHECK( Q.write(1) == true );
    CHECK( Q.write(1) == true );
    CHECK( Q.write(1) == false );

    t.join();
}
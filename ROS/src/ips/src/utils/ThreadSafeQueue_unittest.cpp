#include "tools/unittest/catch.hpp"
#include "ThreadSafeQueue.h"
#include <thread>

using namespace std::chrono_literals;

TEST_CASE("ThreadSafeQueue_PreservesDataOrder") {

    ThreadSafeQueue<int, 17> Q;

    for (int j = 0; j < 11; ++j) {
        for (int i = 5; i < 45; i += 3) {
            Q.write(i);
        }
        for (int i = 5; i < 45; i += 3) {
            int d = 0;
            CHECK( Q.read(d) == true );
            CHECK( d == i );
        }
    }
}


TEST_CASE("ThreadSafeQueue_CloseWhileReading") {

    ThreadSafeQueue<int, 1> Q;

    std::thread t([&](){
        std::this_thread::sleep_for(1ms);
        Q.close();
    });
    
    int d;
    CHECK( Q.write(1) == true );
    CHECK( Q.read(d) == true );
    CHECK( Q.read(d) == false );

    t.join();
}


TEST_CASE("ThreadSafeQueue_CloseWhileWriting") {

    ThreadSafeQueue<int, 2> Q;

    std::thread t([&](){
        std::this_thread::sleep_for(1ms);
        Q.close();
    });
    
    CHECK( Q.write(1) == true );
    CHECK( Q.write(1) == true );
    CHECK( Q.write(1) == false );

    t.join();
}


TEST_CASE("ThreadSafeQueue_ConcurrentReadWrite") {

    ThreadSafeQueue<int, 2> Q;

    std::thread t([&](){
        std::this_thread::sleep_for(1ms);
        Q.write(42);
    });

    int d = 0;
    CHECK( Q.read(d) == true );
    CHECK( d == 42 );

    t.join();
}

TEST_CASE("ThreadSafeQueue_Nonblocking") {

    ThreadSafeQueue<int, 2> Q;



    int d = 0;
    CHECK( Q.read_nonblocking(d) == false );
    CHECK( d == 0 );

    CHECK( Q.write_nonblocking(42) == true );
    CHECK( Q.write_nonblocking(21) == true );
    CHECK( Q.write_nonblocking(123) == false );


    CHECK( Q.read_nonblocking(d) == true );
    CHECK( d == 42 );

    CHECK( Q.read_nonblocking(d) == true );
    CHECK( d == 21 );

    CHECK( Q.read_nonblocking(d) == false );
    CHECK( d == 21 );

}
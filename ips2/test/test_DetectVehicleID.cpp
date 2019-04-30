#include "catch.hpp"
#include "DetectVehicleID.hpp"



TEST_CASE("test_DetectVehicleID")
{
    DetectVehicleID detectVehicleID(
        std::vector<uint8_t>{ 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 },
        std::vector<uint8_t>{ 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 }
    );

    CHECK(0);
}
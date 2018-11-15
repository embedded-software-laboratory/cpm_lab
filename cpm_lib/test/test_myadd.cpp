#include "catch.hpp"
#include "myadd.hpp"

TEST_CASE( "myadd" ) {
    REQUIRE( myadd(3,6) == 9 );
}
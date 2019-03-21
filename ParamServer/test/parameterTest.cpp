#include "catch.hpp"
#include "cpm/Parameter.hpp"
#include "ParameterStorage.hpp"
#include <vector>
#include <string>
#include <algorithm>

TEST_CASE( "parameter_yaml" ) {
    bool b_val = false;
    double d_val = 1.1110001012;
    int i_val = 1119992839;
    std::string s_val = "\t~i1339@77\n33sfg*#'''";
    std::vector<double> d_vals = std::vector<double>({1.1, 1.11222333444, 7.9238349});

    ParameterStorage storage1("test_out.yaml");
    storage1.set_parameter_bool("b1", true);
    storage1.set_parameter_bool("b1", b_val);
    storage1.set_parameter_double("d1", d_val);
    storage1.set_parameter_int("i1", i_val);
    storage1.set_parameter_string("s1", s_val);
    storage1.set_parameter_doubles("d1", d_vals);

    bool bool_value;
    storage1.get_parameter_bool("b1", bool_value);
    CHECK(bool_value == b_val);

    double double_value;
    storage1.get_parameter_double("d1", double_value);
    CHECK(double_value == d_val);

    int int_value;
    storage1.get_parameter_int("i1", int_value);
    CHECK(int_value == i_val);

    std::string string_value;
    storage1.get_parameter_string("s1", string_value);
    CHECK(string_value == s_val);

    std::vector<double> doubles_value;
    storage1.get_parameter_doubles("d1", doubles_value);
    CHECK(doubles_value.size() == d_vals.size());
    for (const double& d : d_vals) {
        CHECK(std::find(doubles_value.begin(), doubles_value.end(), d) != doubles_value.end());
    }

    storage1.storeFile();

    ParameterStorage storage2("test_out.yaml");

    storage2.get_parameter_bool("b1", bool_value);
    CHECK(bool_value == b_val);

    storage2.get_parameter_double("d1", double_value);
    CHECK(double_value == d_val);

    storage2.get_parameter_int("i1", int_value);
    CHECK(int_value == i_val);

    storage2.get_parameter_string("s1", string_value);
    CHECK(string_value == s_val);

    storage2.get_parameter_doubles("d1", doubles_value);
    CHECK(doubles_value.size() == d_vals.size());
    for (const double& d : d_vals) {
        CHECK(std::find(doubles_value.begin(), doubles_value.end(), d) != doubles_value.end());
    }

    storage2.storeFile();
}
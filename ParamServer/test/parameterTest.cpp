#include "catch.hpp"
#include "cpm/Parameter.hpp"
#include "ParameterStorage.hpp"
#include <vector>
#include <string>
#include <algorithm>

TEST_CASE( "parameter_yaml" ) {
    ParameterStorage::Instance().loadFile();
    ParameterStorage::Instance().set_parameter_bool("b1", true);
    ParameterStorage::Instance().set_parameter_bool("b1", false);
    ParameterStorage::Instance().set_parameter_double("d1", 1.1110001012);
    ParameterStorage::Instance().set_parameter_int("i1", 1119992839);
    ParameterStorage::Instance().set_parameter_string("s1", "\t~i1339@77\n33sfg*#'''");
    ParameterStorage::Instance().set_parameter_doubles("d1", std::vector<double>({1.1, 1.11222333444, 7.9238349}));

    bool bool_value;
    ParameterStorage::Instance().get_parameter_bool("b1", bool_value);
    CHECK(bool_value == false);

    double double_value;
    ParameterStorage::Instance().get_parameter_double("d1", double_value);
    CHECK(double_value == 1.1110001012);

    int int_value;
    ParameterStorage::Instance().get_parameter_int("i1", int_value);
    CHECK(int_value == 1119992839);

    std::string string_value;
    ParameterStorage::Instance().get_parameter_string("s1", string_value);
    CHECK(string_value == "\t~i1339@77\n33sfg*#'''");

    std::vector<double> doubles_value;
    ParameterStorage::Instance().get_parameter_doubles("d1", doubles_value);
    CHECK(doubles_value.size() == 3);
    CHECK(std::find(doubles_value.begin(), doubles_value.end(), 1.1) != doubles_value.end());
    CHECK(std::find(doubles_value.begin(), doubles_value.end(), 1.11222333444) != doubles_value.end());
    CHECK(std::find(doubles_value.begin(), doubles_value.end(), 7.9238349) != doubles_value.end());

    ParameterStorage::Instance().storeFile();
}

TEST_CASE("parameter_yaml_storage") {
    ParameterStorage::Instance().loadFile();

    bool bool_value;
    ParameterStorage::Instance().get_parameter_bool("b1", bool_value);
    CHECK(bool_value == false);

    double double_value;
    ParameterStorage::Instance().get_parameter_double("d1", double_value);
    CHECK(double_value == 1.1110001012);

    int int_value;
    ParameterStorage::Instance().get_parameter_int("i1", int_value);
    CHECK(int_value == 1119992839);

    std::string string_value;
    ParameterStorage::Instance().get_parameter_string("s1", string_value);
    CHECK(string_value == "\t~i1339@77\n33sfg*#'''");

    std::vector<double> doubles_value;
    ParameterStorage::Instance().get_parameter_doubles("d1", doubles_value);
    CHECK(doubles_value.size() == 3);
    CHECK(std::find(doubles_value.begin(), doubles_value.end(), 1.1) != doubles_value.end());
    CHECK(std::find(doubles_value.begin(), doubles_value.end(), 1.11222333444) != doubles_value.end());
    CHECK(std::find(doubles_value.begin(), doubles_value.end(), 7.9238349) != doubles_value.end());

    ParameterStorage::Instance().storeFile();
}
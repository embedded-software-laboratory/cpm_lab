#include "ParameterServer.hpp"
#include <vector>
#include <string>
#include <rti/core/QosProviderParams.hpp>
#include <dds/core/QosProvider.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    rti::core::QosProviderParams params;
    params.ignore_environment_profile(true);
    dds::core::QosProvider::Default()->default_profile(
       rti::core::USE_DDS_DEFAULT_QOS_PROFILE);
    dds::core::QosProvider::Default()->default_provider_params(params);
    dds::core::QosProvider::Default()->reload_profiles();

    ParameterServer server;

    server.set_value("t1", false);
    server.set_value("t2", true);

    sleep(5);

    vector<int32_t> ints = {1, 2, 3};
    server.set_value("t3", ints);

    sleep(5);

    server.set_value("t4", 7.777777);

    sleep(5);

    server.set_value("t5", "Hey");
    vector<std::string> strings = {"Dies", "ist", "ein", "Test"};
    server.set_value("t6", strings);

    sleep(100);
}
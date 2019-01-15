#include "ParameterServer.hpp"
#include <vector>
#include <string>
#include <rti/core/QosProviderParams.hpp>
#include <dds/core/QosProvider.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    ParameterServer server;

    server.set_value("t1", false);
    server.set_value("t2", true);

    sleep(5);

    vector<int32_t> ints = {1, 2, 3};
    server.set_value("t3", ints);

    sleep(5);

    server.set_value("t4", 7.777777);

    sleep(5);

    std::string val = "Hey";
    server.set_value("t5", val);

    sleep(100);
}
/**
 * \class main.cpp
 * \brief This class exists for test purposes only
 */

#include "cpm/Parameter.hpp"
#include "ParameterStorage.hpp"

int main(int argc, char *argv[])
{
    ParameterStorage::Instance().loadFile();

    ParameterStorage::Instance().storeFile();

    return 0;
}
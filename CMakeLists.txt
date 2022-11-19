cmake_minimum_required(VERSION 3.12)
project(BotaSerialDriver)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)
include_directories(examples)

add_executable(BotaSerialDriver
        examples/read_sensor_linux.cpp
        BotaForceTorqueSensorComm.cpp
        BotaForceTorqueSensorComm.h)

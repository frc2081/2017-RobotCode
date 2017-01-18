cmake_minimum_required(VERSION 2.8)
include(frc.cmake)

project(2017-RobotCode)

# Team number
set(TEAM_NUMBER 2081)

# Robot target
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(SOURCE_FILES
        src/Robot.h
        src/Robot.cpp
        src/cntl.h
        src/cntl.cpp
        src/swervelib.h
        src/swervelib.cpp)
add_frc_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Deploy target
add_frc_deploy(deploy ${TEAM_NUMBER} $<TARGET_FILE:${PROJECT_NAME}>)
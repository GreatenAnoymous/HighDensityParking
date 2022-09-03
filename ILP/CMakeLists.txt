project(mrmp)


cmake_minimum_required(VERSION 3.16)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )

set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG  -O3")

set(GUROBI_INC /opt/gurobi912/linux64/include/)
set(GUROBI_LIB "/opt/gurobi912/linux64/lib/libgurobi_c++.a" "/opt/gurobi912/linux64/lib/libgurobi91.so")
set(ORTOOLS_LIB "/usr/local/lib/libortools.so")

file(GLOB DDM_src "./main.cpp" "./common.cpp" "./ILP.cpp" "./ecbs.cpp")



add_executable(main ${DDM_src})
# add_executable(drp test.cpp benchmark.cpp drp.cpp)
target_compile_features(main PUBLIC cxx_std_17)
# target_include_directories(drp_exe  PUBLIC "./libMultiRobotPlanning/include/" )
target_include_directories(main  PUBLIC ${GUROBI_INC} )
target_link_libraries(main PUBLIC  stdc++fs ${ORTOOLS_LIB} ${GUROBI_LIB})



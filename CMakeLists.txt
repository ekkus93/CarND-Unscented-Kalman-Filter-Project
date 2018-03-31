project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/ukf.cpp src/main.cpp src/tools.cpp src/sensor_data.cpp src/sensor_data.h)

# gtest 
set(test_sources test/gtest_main.cpp src/ukf.cpp src/tools.cpp src/sensor_data.cpp src/sensor_data.h)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.19.2/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(UnscentedKF ${sources})

# gtest 
add_executable(Test_UnscentedKF ${test_sources})
set_target_properties(Test_UnscentedKF PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(UnscentedKF z ssl uv uWS)

# gtest 
target_link_libraries(Test_UnscentedKF gtest gmock z ssl uv uWS)
cmake_minimum_required(VERSION 3.10)
project(CoviewContext)

# Set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)

# CSF library
set(CSF_LIBRARY "${CMAKE_SOURCE_DIR}/3rdParty/libCSF.a")

# pcl library
# set(PCL_DIR "/home/dxz/pcl-1.13/share/pcl-1.13")
find_package(PCL 1.8 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV library
find_package(OpenCV REQUIRED)

# Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Boost library
find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(./include)

# executable target
add_executable(demo src/demo.cpp src/CoviewContext.cpp include/ScanContext/Scancontext.cpp)

if(OpenMP_CXX_FOUND)
    target_link_libraries(demo PUBLIC OpenMP::OpenMP_CXX)
endif()

# libraries to the executable
target_link_libraries(demo PUBLIC ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_INCLUDE_DIR} ${Boost_LIBRARIES} ${CSF_LIBRARY})
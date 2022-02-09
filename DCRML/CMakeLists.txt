cmake_minimum_required(VERSION 3.10)
project(teaser_cpp_ply)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE  "-Wall -W  -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -W -g -ggdb -O0")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        /usr/local/include/eigen3/
        MC-RANSAC/include
)

# Change this line to include your own executable file

add_library(utils
        registration.cpp
        utility.cpp
        )

add_library(toronot_lib
        MC-RANSAC/src/localization.cpp
        MC-RANSAC/src/association.cpp
        MC-RANSAC/src/features.cpp
        MC-RANSAC/src/radar_utils.cpp
        )

target_link_libraries(utils
        ${OpenCV_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
        toronot_lib
        )

add_executable(DCRML DCRML.cpp)
target_link_libraries(DCRML
        ${OpenCV_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
        toronot_lib  utils
        )

# Link to teaserpp & Eigen3
#target_link_libraries(teaser_cpp_ply Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io)

# Copy the data files to build directory
file(COPY ../example_data/
        DESTINATION ./example_data/
        FILES_MATCHING
        PATTERN *.ply)

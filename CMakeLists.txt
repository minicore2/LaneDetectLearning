#####################################
cmake_minimum_required (VERSION 3.0) 
project (LaneDetectLearning)
add_compile_options(-std=c++11)
add_library(LANE_CONSTANT_LIBRARIES lane_constant_class.cpp)
add_library(LANE_CONSTANT_TEMPLATES lane_constant_templates.cpp)
add_library(RESULT_VALUES_LIBRARIES result_values_class.cpp)
add_library(LANE_DETECT_LIBRARIES lane_detect_processor.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable (main main.cpp)
target_link_libraries(main
	${OpenCV_LIBS}
	LANE_CONSTANT_LIBRARIES
	LANE_CONSTANT_TEMPLATES
	RESULT_VALUES_LIBRARIES
	LANE_DETECT_LIBRARIES
)
#####################################

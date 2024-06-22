add_library(rrt rrt.h rrt.cpp)
target_include_directories(rrt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rrt fssimplewindow ysbitmapfont 2dgrid base)
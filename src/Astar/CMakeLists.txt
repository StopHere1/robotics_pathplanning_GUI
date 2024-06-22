add_library(astar astar.h astar.cpp)
target_include_directories(astar PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(astar fssimplewindow ysbitmapfont 2dgrid base)
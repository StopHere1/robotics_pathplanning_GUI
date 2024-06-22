add_library(dijkstra dijkstra.h dijkstra.cpp)
target_include_directories(dijkstra PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dijkstra fssimplewindow ysbitmapfont 2dgrid base astar)
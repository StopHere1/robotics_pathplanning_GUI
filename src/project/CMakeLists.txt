add_executable(project MACOSX_BUNDLE main.cpp)

#target_link_libraries(project fssimplewindow ysbitmapfont 2dgrid base dijkstra)
target_link_libraries(project 2dgrid base dijkstra astar rrt prm application)
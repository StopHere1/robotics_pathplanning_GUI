add_library(application application.h application.cpp)
target_include_directories(application PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(application fssimplewindow ysbitmapfont 2dgrid base dijkstra astar rrt prm rscurve ysbitmap)
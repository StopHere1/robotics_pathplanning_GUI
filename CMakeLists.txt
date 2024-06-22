CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

Project(ROBOCONSTRUCTION)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(../public ${CMAKE_BINARY_DIR}/public)
add_subdirectory(../MMLPlayer/ym2612 ${CMAKE_BINARY_DIR}/ym2612)
add_subdirectory(../MMLPlayer/mmlplayer ${CMAKE_BINARY_DIR}/mmlplayer)

#add_subdirectory(src)
add_subdirectory(src/base)
add_subdirectory(src/2dgrid)
add_subdirectory(src/dijkstra)
add_subdirectory(src/astar)
add_subdirectory(src/rrt)
add_subdirectory(src/prm)
add_subdirectory(src/application)
add_subdirectory(src/project)
add_subdirectory(src/rscurve)
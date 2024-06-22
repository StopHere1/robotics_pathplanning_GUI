add_library(2dgrid gridbase.h lattice2d.h space2d.h space2d.cpp)
target_link_libraries(2dgrid base)
target_include_directories(2dgrid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
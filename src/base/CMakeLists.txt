add_library(base parameter.cpp parameter.h renderbase.h renderbase.cpp base_algorithm.h)

target_link_libraries(base fssimplewindow ysbitmapfont 2dgrid)
target_include_directories(base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
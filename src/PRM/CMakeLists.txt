add_library(prm prm.h prm.cpp)
target_include_directories(prm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(prm fssimplewindow ysbitmapfont 2dgrid base)
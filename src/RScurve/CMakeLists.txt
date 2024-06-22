add_library(rscurve rscurve.h rscurve.cpp rsmotion.h rsmotion.cpp rs_curve_planner.h wl_path.h)
target_include_directories(rscurve PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rscurve fssimplewindow ysbitmapfont 2dgrid base)
FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

add_subdirectory("leap")
add_subdirectory("Test_C5G")
add_subdirectory("Test_scanBins")
add_subdirectory("DemoC5GOpen")
add_subdirectory("Test_condition_variables")
add_subdirectory("Taratura_shelf")
add_subdirectory("Sandbox/provamain/")
add_subdirectory("Sandbox/GiorgioT/MyLinemodPlusRender/")
add_subdirectory("Sandbox/TCPtest/")
add_subdirectory("OpenNI_Streaming_pleasedontlookatme/")
add_subdirectory("Test_Giorgio/")
add_subdirectory("Test_depth/")

#include(${OpenCV_CONFIG_PATH}/OpenCVConfig.cmake)
#Set all of the opencv contrib stuff not to compile, EXCEPT for rgbd
#FILE(GLOB opencv_contrib_modules opencv_contrib/modules/*)
#MESSAGE("OPENCV CONTRIB MODULES:")
#MESSAGE("BLARALERO ${opencv_contrib_modules}")
#FOREACH(nome IN LISTS opencv_contrib_modules)
#  get_filename_component(barename ${nome} NAME)
#  SET("BUILD_opencv_${barename}" OFF)
#ENDFOREACH(nome IN LISTS opencv_contrib_modules)
#
#SET("BUILD_opencv_rgbd" TRUE)
#
add_subdirectory("OpenCV_RGBD/")

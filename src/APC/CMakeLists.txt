FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(assimp REQUIRED)
add_library(workorder SHARED Order.cpp )
SET_TARGET_PROPERTIES(workorder PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(apc SHARED Shelf.cpp ScanBins.cpp OrderBin.cpp UpdateBins.cpp Grasper.cpp ReadWorkOrder.cpp UpdateBins.cpp Robot.cpp)
target_link_libraries(apc camera recognition robotdata workorder c5g_misc c5g)

add_library(apc_main SHARED APC.cpp)
#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( apc PROPERTIES COMPILE_FLAGS "-fPIC" )

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( apc_main PROPERTIES COMPILE_FLAGS "-fPIC" )

target_link_libraries(apc_main apc camera recognition robotdata ${PCL_LIBRARIES} ${assimp_LIBRARIES})

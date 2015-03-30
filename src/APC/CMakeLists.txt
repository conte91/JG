add_library(apc Shelf.cpp Order.cpp ScanBins.cpp OrderBin.cpp UpdateBins.cpp Grasper.cpp ReadWorkOrder.cpp)
target_link_libraries(apc camera robotdata)

add_library(apc_main APC.cpp)
#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( apc PROPERTIES COMPILE_FLAGS "-fPIC" )

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( apc_main PROPERTIES COMPILE_FLAGS "-fPIC" )

target_link_libraries(apc_main apc camera)

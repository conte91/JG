FIND_PACKAGE(PythonInterp 2 REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)

MAYBE_FIND(OpenCV)
MAYBE_FIND(Eigen)

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
PYTHON_ADD_MODULE(_transformations transformations.c )

COPY_TO_LIB(gripper.py)
COPY_TO_LIB(transformations.py)

add_library(shapes SHARED Shape.cpp Sphere.cpp Cuboid.cpp ComposedShape.cpp Cylinder.cpp ShapeBuilder.cpp)
SET_TARGET_PROPERTIES( shapes PROPERTIES COMPILE_FLAGS "-fPIC" )
target_link_libraries(shapes ${OpenCV_LIBRARIES})


add_library(gripper SHARED GripperModel.cpp )
SET_TARGET_PROPERTIES(gripper PROPERTIES COMPILE_FLAGS "-fPIC")
target_link_libraries(gripper shapes grasping)

add_library(grasping SHARED GraspPose.cpp Object.cpp PoseFactory.cpp)
SET_TARGET_PROPERTIES( grasping PROPERTIES COMPILE_FLAGS "-fPIC" )

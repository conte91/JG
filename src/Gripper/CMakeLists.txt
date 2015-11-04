FIND_PACKAGE(PythonInterp 2 REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)

MAYBE_FIND(Eigen)

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
PYTHON_ADD_MODULE(_transformations transformations.c )

COPY_TO_LIB(gripper.py)
COPY_TO_LIB(transformations.py)

add_library(shapes SHARED Shape.cpp Sphere.cpp Cuboid.cpp ComposedShape.cpp Cylinder.cpp)
SET_TARGET_PROPERTIES( shapes PROPERTIES COMPILE_FLAGS "-fPIC" )

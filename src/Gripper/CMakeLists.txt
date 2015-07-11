FIND_PACKAGE(PythonInterp 2 REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
PYTHON_ADD_MODULE(_transformations transformations.c )

COPY_TO_LIB(gripper.py)
COPY_TO_LIB(transformations.py)

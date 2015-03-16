IF(NOT(DEFINED NO_EORL))

find_package(eORL REQUIRED)

include_directories(${eORL_INCLUDE_DIRS})

add_executable(main
keyboard.c
loopConsole.c
test_crcopen_util.c
user_callback.c
demo_c5gopen.c
)

target_link_libraries(main m pthread ${eORL_LIBRARIES})

ENDIF(NOT(DEFINED NO_EORL))

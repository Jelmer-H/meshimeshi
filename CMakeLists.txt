cmake_minimum_required(VERSION 3.16)
project(meshimeshi C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Strict core: the protocol core must build without warnings.
add_compile_options(-Wall -Wextra -Werror)

# ---- protocol core as shared library ---------------------------------
# The simulator (sim/sim1.py) calls this via ctypes; firmware platforms
# link the same source statically. No dynamic allocation in the core (13b).
add_library(mimeshi_shared SHARED core/core.c core/core.h)
set_target_properties(mimeshi_shared PROPERTIES OUTPUT_NAME mimeshi)
# the shared .so is also loaded by the simulator (all nodes of a scenario
# in one process): the larger bound pool, as in the README gcc line
target_compile_definitions(mimeshi_shared PRIVATE MC_BOUND_POOL=8192)
target_include_directories(mimeshi_shared PUBLIC ${CMAKE_SOURCE_DIR})

# Static variant for firmware builds.
add_library(mimeshi_static STATIC core/core.c core/core.h)
target_include_directories(mimeshi_static PUBLIC ${CMAKE_SOURCE_DIR})

# ---- test programs ---------------------------------------------------------
# Scenario 1 on the C core itself (without Python). Compile and run with
#   ctest --test-dir build
enable_testing()
add_executable(test_link_test tests/test_link_test.c)
add_executable(test_table_chunk tests/test_table_chunk.c)
# Asserts are the actual test checks: keep them active even in
# RelWithDebInfo (NDEBUG would compile them away and the test would
# assert nothing).
target_compile_options(test_link_test PRIVATE -UNDEBUG)
target_link_libraries(test_link_test mimeshi_static)
add_test(NAME link_test COMMAND test_link_test)
target_compile_options(test_table_chunk PRIVATE -UNDEBUG)
target_link_libraries(test_table_chunk mimeshi_static)
add_test(NAME table_chunk COMMAND test_table_chunk)

# ---- platform layers (stubs; per platform only the hardware interface) ----
add_subdirectory(platform/host)

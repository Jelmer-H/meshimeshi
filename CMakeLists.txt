cmake_minimum_required(VERSION 3.16)
project(meshimeshi C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Strenge kern: de protocolkern moet zonder waarschuwingen bouwen.
add_compile_options(-Wall -Wextra -Werror)

# ---- protocolkern als gedeelde bibliotheek ---------------------------------
# De simulator (sim/sim1.py) roept deze aan via ctypes; firmware-platforms
# linken dezelfde bron statisch. Geen dynamische allocatie in de kern (13b).
add_library(mimeshi_shared SHARED core/core.c core/core.h)
set_target_properties(mimeshi_shared PROPERTIES OUTPUT_NAME mimeshi)
target_include_directories(mimeshi_shared PUBLIC ${CMAKE_SOURCE_DIR})

# Statische variant voor firmware-builds.
add_library(mimeshi_static STATIC core/core.c core/core.h)
target_include_directories(mimeshi_static PUBLIC ${CMAKE_SOURCE_DIR})

# ---- testprogramma's -------------------------------------------------------
# Scenario 1 op de C-kern zelf (zonder Python):Compileer en draai met
#   ctest --test-dir build
enable_testing()
add_executable(test_koppeltest tests/test_koppeltest.c)
target_link_libraries(test_koppeltest mimeshi_static)
add_test(NAME koppeltest COMMAND test_koppeltest)

# ---- platformlagen (stubs; per platform alleen de hardware-interface) -----
add_subdirectory(platform/host)

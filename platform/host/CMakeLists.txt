# host platform: dummy hardware layer for workstation development.
# Provides the four callbacks (tx / timer / channel switch / RNG) at stdout level.
# Used by test_link_test and later by a 'console radio'.
add_library(platform_host STATIC host_radio.c)
target_include_directories(platform_host PUBLIC ../..)

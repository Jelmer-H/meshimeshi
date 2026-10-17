# host-platform: dummy hardware-laag voor ontwikkeling op een werkstation.
# Levert de vier callbacks (tx / timer / kanaalwissel / RNG) op stdout-niveau.
# Wordt gebruikt door test_koppeltest en later door een 'console-radio'.
add_library(platform_host STATIC host_radio.c)
target_include_directories(platform_host PUBLIC ../..)

add_library(openMVG_stl INTERFACE)
target_link_libraries(openMVG_stl INTERFACE ${LOG_LIB})

UNIT_TEST(openMVG split "openMVG_stl")
UNIT_TEST(openMVG dynamic_bitset "openMVG_stl")

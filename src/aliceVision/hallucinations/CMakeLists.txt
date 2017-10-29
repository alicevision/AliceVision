set(SOURCES_files_hallucinations
  hallucinations.cpp
  hallucinations.hpp
)
source_group("hallucinations" FILES ${SOURCES_files_hallucinations})

add_library(mv_hallucinations ${SOURCES_files_hallucinations})
target_link_libraries(mv_hallucinations PUBLIC mv_structures mv_mesh ${Boost_LIBRARIES})

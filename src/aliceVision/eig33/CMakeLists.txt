set(SOURCES_files_eig33
  eig33.cpp
  eig33.hpp
)
source_group("eig33" FILES ${SOURCES_files_eig33})

set(SOURCES_eig33
  ${SOURCES_files_eig33}
)

add_library(eig33 ${SOURCES_files_eig33})

set_target_properties(eig33 PROPERTIES FOLDER 3rdParty)
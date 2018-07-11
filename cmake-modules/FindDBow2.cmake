
#set(DBoW2_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/DBoW2/")
#set(DBoW2_ROOT "C:/slam_utils/DBoW2")

if (DBOW2_INCLUDE_DIR AND DBOW2_LIBRARIES)
  set(DBOW2_FIND_QUIETLY TRUE)
endif (DBOW2_INCLUDE_DIR AND DBOW2_LIBRARIES)

find_path(DBOW2_INCLUDE_DIR
  NAMES
  DBoW2/FORB.h
  PATHS
  ${DBoW2_ROOT}/include/
  $ENV{DBOW2DIR}
  ${INCLUDE_INSTALL_DIR}
)

find_library(DBOW2_LIBRARY DBoW2 PATHS ${DBoW2_ROOT}/lib/ $ENV{DBOW2DIR} ${LIB_INSTALL_DIR})
set(DBOW2_LIBRARIES ${DBOW2_LIBRARY})


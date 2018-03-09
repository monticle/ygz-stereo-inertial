
#set(FAST_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/fast/")
set(FAST_ROOT "C:/slam_utils/fast")

if (FAST_INCLUDE_DIR AND FAST_LIBRARIES)
  set(FAST_FIND_QUIETLY TRUE)
endif (FAST_INCLUDE_DIR AND FAST_LIBRARIES)

find_path(FAST_INCLUDE_DIR
  NAMES
  fast/fast.h
  PATHS
  ${FAST_ROOT}/include/
  $ENV{FASTDIR}
  ${INCLUDE_INSTALL_DIR}
)

find_library(FAST_LIBRARY fast PATHS ${FAST_ROOT}/lib/ $ENV{FASTDIR} ${LIB_INSTALL_DIR})
set(FAST_LIBRARIES ${FAST_LIBRARY})

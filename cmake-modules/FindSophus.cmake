
#set(Sophus_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/Sophus/")
set(Sophus_ROOT "C:/slam_utils/sources/Sophus-Windows")

if (SOPHUS_INCLUDE_DIR)
  set(SOPHUS_FIND_QUIETLY TRUE)
endif (SOPHUS_INCLUDE_DIR)

find_path(SOPHUS_INCLUDE_DIR
  NAMES
  sophus/se3.hpp
  PATHS
  ${Sophus_ROOT}/
  $ENV{SOPHUSDIR}
  ${INCLUDE_INSTALL_DIR}
)


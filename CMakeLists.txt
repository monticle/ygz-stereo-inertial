cmake_minimum_required( VERSION 2.8 )
project(YGZ-Stereo)

# uncommont this to use release
set( CMAKE_BUILD_TYPE "Release" )

# make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()


if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
elseif(NOT WIN32)
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}', turn on the compiler optimization")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wall -march=native -O3 -Wno-reorder -pthread -fopenmp")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native -O3 -pthread -fopenmp")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_STANDARD 11) #C++ 11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #..without compiler extensions like gnu++11

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(NOT MSVC)
	if(COMPILER_SUPPORTS_CXX11)
	   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	   add_definitions(-DCOMPILEDWITHC11)
	   message(STATUS "Using flag -std=c++11.")
	else(COMPILER_SUPPORTS_CXX11)
	   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
	endif(COMPILER_SUPPORTS_CXX11)
else(NOT MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif(NOT MSVC)

# arm and intel cpu configurations
IF(WIN32)
  SET(IS_ARM FALSE)
  add_definitions(-DNOMINMAX)
ELSE()
  EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
  if( ${ARCHITECTURE} MATCHES "aarch64" )
  	set( IS_ARM TRUE )
  else()
  	SET(IS_ARM FALSE)
  endif()
ENDIF()
IF(IS_ARM)
   #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a") #this is for 32bit
  ADD_DEFINITIONS(-DPNG_ARM_NEON_CHECK_SUPPORTED)
ELSEIF(NOT WIN32)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -march=native")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
include(cotire)
set(COTIRE_MINIMUM_NUMBER_OF_TARGET_SOURCES 1)
set (COTIRE_PCH_MEMORY_SCALING_FACTOR "400")

# thirdparty libs 
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    message(STATUS "Don't get Opencv 3.0, looking for OpenCV 2.4.")
   find_package(OpenCV 2.4 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(glew REQUIRED)
find_package(Pangolin REQUIRED)

# to show 'include' in QtCreator
#FILE(GLOB_RECURSE INC_ALL "*/*.h")
#add_custom_target(headers SOURCES ${INC_ALL})

if(WIN32)
	find_package(DBoW2 REQUIRED)
	find_package(g2o REQUIRED)
	find_package(fast REQUIRED)
	#find_package(Sophus REQUIRED)
	
	find_package( glog REQUIRED )
	set(GLOG_LIBRARY glog::glog) #https://github.com/google/glog/issues/155
else()

	find_package( Glog3 REQUIRED )
	set(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/)
	set(G2O_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)
	set(DBOW2_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/)
	set(DBOW2_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
	set(FAST_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/fast/include)
	set(FAST_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/fast/build/libfast.so)
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIR}
${DBOW2_INCLUDE_DIR}
${SOPHUS_INCLUDE_DIR}
${FAST_INCLUDE_DIR}
${GLEW_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)


# thirdparty
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${G2O_LIBRARIES}
        ${DBOW2_LIBRARIES}
        ${FAST_LIBRARIES}
		${GLEW_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${GLOG_LIBRARY}
)
message("GLOG_INCLUDE_DIRS: ${GLOG_INCLUDE_DIRS}")
message("GLOG_LIBRARY: ${GLOG_LIBRARY}")
# subdirectories
include_directories(
        util/include
        cv/include
        common/include
        backend/include
        system/include
)


add_subdirectory(util)

add_subdirectory(common)

add_subdirectory(cv)

add_subdirectory(backend)

add_subdirectory(system)

set(YGZ_LIBS ygz-common ygz-cv ygz-util ygz-backend ygz-system)
# build examples
#add_subdirectory( examples )

add_subdirectory( leadsense )

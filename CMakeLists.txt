# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# TRACKER Project configuration
PROJECT(XTION_GRABBER)
SET(CMAKE_CXX_FLAGS "-std=c++0x")
SET(CMAKE_LD_FLAGS "-lz")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# core libraries
option(BUILD_GRABBER "Build Asus Xtion grabber libraries" YES)
option(BUILD_GRABBER_DEFS "Build GRABBER Defs library" YES)
mark_as_advanced(BUILD_GRABBER_DEFS)
option(BUILD_OPENNI_WRAPPER "Build OpenNI Wrapper library" YES)
mark_as_advanced(BUILD_OPENNI_WRAPPER)
option(BUILD_GRABBER_DEV "Build Grabber Dev library" YES)
mark_as_advanced(BUILD_GRABBER_DEV)

# demos
option(BUILD_GRABBER_DEMO "Build GRABBer demonstration programs and libraries" YES)
option(BUILD_GRABBER_DEMO_BASIC "Build basic Grabber demonstration programs and libraries" YES)
mark_as_advanced(BUILD_GRABBER_DEMO_BASIC)
option(BUILD_GRABBER_DEMO_REC "Build Grabber recorder program and libraries" YES)
mark_as_advanced(BUILD_GRABBER_DEMO_REC)
#additional dependencies
# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

#Eigen
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#OpenNI
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2")

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

if (MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

###############################################################################
#
# PUTSLAM tinyXML2 library
#
###############################################################################

add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

###############################################################################
#
# TRACKER defs library
#
###############################################################################

if(BUILD_GRABBER AND BUILD_GRABBER_DEFS)
        file(GLOB DEFS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Defs/tracker_defs.h"
        )
        ADD_LIBRARY(GRABBER_DEFS STATIC ${DEFS_HEADERS})
        set_target_properties(GRABBER_DEFS PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS TRACKERDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Defs/)

endif(BUILD_GRABBER AND BUILD_GRABBER_DEFS)

###############################################################################
#
# TRACKER Grabber library
#
###############################################################################

if(BUILD_GRABBER AND BUILD_OPENNI_WRAPPER)
        file(GLOB NI_WRAPPER_SOURCES
            "${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/OpenNIWrapper.cpp"
        )
        file(GLOB NI_WRAPPER_HEADERS
            "${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2/OpenNIWrapper.h"
        )

        ADD_LIBRARY(OpenNIWrapper STATIC ${NI_WRAPPER_SOURCES} ${NI_WRAPPER_HEADERS})
        TARGET_LINK_LIBRARIES(OpenNIWrapper OpenNI2 opencv_imgproc ${PCL_LIBRARIES})
        INSTALL(TARGETS OpenNIWrapper RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${NI_WRAPPER_HEADERS} DESTINATION include/Grabber/Grabber/)
endif(BUILD_GRABBER AND BUILD_OPENNI_WRAPPER)

###############################################################################
#
# TRACKER Grabber library
#
###############################################################################

if(BUILD_GRABBER AND BUILD_GRABBER_DEV)
        file(GLOB GRABBER_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Grabber/*.cpp"
        )
        file(GLOB GRABBER_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Grabber/*.h"
        )

        ADD_LIBRARY(GrabberDev STATIC ${GRABBER_SOURCES} ${GRABBER_HEADERS})
        TARGET_LINK_LIBRARIES(GrabberDev OpenNIWrapper OpenNI tinyxml2 boost_system)
        INSTALL(TARGETS GrabberDev RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${GRABBER_HEADERS} DESTINATION include/Grabber/Grabber/)
endif(BUILD_GRABBER AND BUILD_GRABBER_DEV)

###############################################################################
#
# GRABBER DEMO executables
#
###############################################################################

if(BUILD_GRABBER_DEMO AND BUILD_GRABBER_DEMO_BASIC)

        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(Demo GrabberDev ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
        INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_GRABBER_DEMO AND BUILD_GRABBER_DEMO_BASIC)


###############################################################################
#
# GRABBER RECORDER DEMO executables
#
###############################################################################

if(BUILD_GRABBER_DEMO AND BUILD_GRABBER_DEMO_REC)

        SET(DEMO_SOURCES ./recorder.cpp)
        ADD_EXECUTABLE(recorder ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(recorder GrabberDev ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
        INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_GRABBER_DEMO AND BUILD_GRABBER_DEMO_REC)

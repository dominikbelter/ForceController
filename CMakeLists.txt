# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# TRACKER Project configuration
PROJECT(ForceController)
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_LD_FLAGS "-lz")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty")

# core libraries
option(BUILD_CONTROLLER "Build Force Controller libraries for walking robot" YES)
option(BUILD_CONTROLLER_DEFS "Build Force Controller Defs library" YES)
mark_as_advanced(BUILD_CONTROLLER_DEFS)
option(BUILD_CONTROLLER_BOARD "Build Controller Board library" YES)
mark_as_advanced(BUILD_CONTROLLER_BOARD)
option(BUILD_CONTROLLER_KINEMATIC "Build Kinematic library" YES)
mark_as_advanced(BUILD_CONTROLLER_KINEMATIC)
option(BUILD_CONTROLLER_LEG "Build Leg controller library" YES)
mark_as_advanced(BUILD_CONTROLLER_LEG)
option(BUILD_CONTROLLER_ROBOT "Build Controller Robot library" YES)
mark_as_advanced(BUILD_CONTROLLER_ROBOT)
option(BUILD_CONTROLLER_VISUALIZATION "Build Visualization library" YES)
mark_as_advanced(BUILD_CONTROLLER_VISUALIZATION)

# demos
option(BUILD_CONTROLLER_DEMO "Build Controller demonstration programs and libraries" YES)
option(BUILD_CONTROLLER_DEMO_BASIC "Build basic Controller demonstration programs and libraries" YES)
mark_as_advanced(BUILD_CONTROLLER_DEMO_BASIC)
option(BUILD_CONTROLLER_DEMO_BOARD "Build Board program and libraries" YES)
mark_as_advanced(BUILD_CONTROLLER_DEMO_BOARD)
option(BUILD_CONTROLLER_DEMO_KINEMATIC "Build Kinematic program and libraries" YES)
mark_as_advanced(BUILD_CONTROLLER_DEMO_KINEMATIC)
option(BUILD_CONTROLLER_DEMO_LEG "Build Leg Controller program and libraries" YES)
mark_as_advanced(BUILD_CONTROLLER_DEMO_LEG)
option(BUILD_CONTROLLER_DEMO_ROBOT "Build Robot Controller program and libraries" YES)
mark_as_advanced(BUILD_CONTROLLER_DEMO_ROBOT)
option(BUILD_CONTROLLER_DEMO_VISUALIZATION "Build Visualization program and libraries" YES)
mark_as_advanced(BUILD_CONTROLLER_DEMO_VISUALIZATION)

#additional dependencies
# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
# FIND_PACKAGE(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )

#Eigen
add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# PCL
# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

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
# tinyXML2 library
#
###############################################################################

add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

###############################################################################
#
# Dynamixel library
#
###############################################################################
add_library(dynamixel STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dynamixel.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dxl_hal.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dynamixel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dxl_hal.cpp)


###############################################################################
#
# Irrlicht library
#
###############################################################################
FIND_PACKAGE(X11)
FIND_PACKAGE(ZLIB)

INCLUDE_DIRECTORIES(
   "/usr/include/irrlicht"
   ${X11_INCLUDE_DIR}
   ${ZLIB_INCLUDE_DIR}
)

###############################################################################
#
# defs library
#
###############################################################################

if(BUILD_CONTROLLER AND BUILD_CONTROLLER_DEFS)
        file(GLOB DEFS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/defs/defs.h"
        )
        ADD_LIBRARY(CONTROLLER_DEFS STATIC ${DEFS_HEADERS})
        set_target_properties(CONTROLLER_DEFS PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS TRACKERDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Defs/)

endif(BUILD_CONTROLLER AND BUILD_CONTROLLER_DEFS)

###############################################################################
#
# CONTROLLER Board library
#
###############################################################################

if(BUILD_CONTROLLER AND BUILD_CONTROLLER_BOARD)
        file(GLOB BOARD_SOURCES
            "${CMAKE_SOURCE_DIR}/src/board/*.cpp"
        )
        file(GLOB BOARD_HEADERS
            "${CMAKE_SOURCE_DIR}/include/board/*.h"
        )

        ADD_LIBRARY(Board STATIC ${BOARD_SOURCES} ${BOARD_HEADERS})
#         TARGET_LINK_LIBRARIES(OpenNIWrapper OpenNI2 opencv_imgproc ${PCL_LIBRARIES})
        INSTALL(TARGETS Board RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${BOARD_HEADERS} DESTINATION include/Board/)
endif(BUILD_CONTROLLER AND BUILD_CONTROLLER_BOARD)

###############################################################################
#
# Kinematic Board library
#
###############################################################################

if(BUILD_CONTROLLER AND BUILD_CONTROLLER_KINEMATIC)
        file(GLOB KINEMATIC_SOURCES
            "${CMAKE_SOURCE_DIR}/src/kinematic/*.cpp"
        )
        file(GLOB KINEMATIC_HEADERS
            "${CMAKE_SOURCE_DIR}/include/kinematic/*.h"
        )

        ADD_LIBRARY(Kinematic STATIC ${KINEMATIC_SOURCES} ${KINEMATIC_HEADERS})
        TARGET_LINK_LIBRARIES(Kinematic tinyxml2)
        INSTALL(TARGETS Kinematic RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${KINEMATIC_HEADERS} DESTINATION include/Kinematic/)
endif(BUILD_CONTROLLER AND BUILD_CONTROLLER_KINEMATIC)

###############################################################################
#
# CONTROLLER Leg library
#
###############################################################################

if(BUILD_CONTROLLER AND BUILD_CONTROLLER_LEG)
        file(GLOB LEG_SOURCES
            "${CMAKE_SOURCE_DIR}/src/legModel/*.cpp"
        )
        file(GLOB LEG_HEADERS
            "${CMAKE_SOURCE_DIR}/include/legModel/*.h"
        )

        ADD_LIBRARY(Leg STATIC ${LEG_SOURCES} ${LEG_HEADERS})
        TARGET_LINK_LIBRARIES(Leg Kinematic)
        INSTALL(TARGETS Leg RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${LEG_HEADERS} DESTINATION include/legControl/)
endif(BUILD_CONTROLLER AND BUILD_CONTROLLER_LEG)

###############################################################################
#
# CONTROLLER Robot library
#
###############################################################################

if(BUILD_CONTROLLER AND BUILD_CONTROLLER_ROBOT)
        file(GLOB ROBOT_SOURCES
            "${CMAKE_SOURCE_DIR}/src/robotModel/*.cpp"
        )
        file(GLOB ROBOT_HEADERS
            "${CMAKE_SOURCE_DIR}/include/robotModel/*.h"
        )

        ADD_LIBRARY(Robot STATIC ${ROBOT_SOURCES} ${ROBOT_HEADERS})
	TARGET_LINK_LIBRARIES(Robot Leg Kinematic)
        INSTALL(TARGETS Robot RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${ROBOT_HEADERS} DESTINATION include/robotControl/)
endif(BUILD_CONTROLLER AND BUILD_CONTROLLER_ROBOT)

###############################################################################
#
# CONTROLLER Visualization library
#
###############################################################################

if(BUILD_CONTROLLER AND BUILD_CONTROLLER_VISUALIZATION)
        file(GLOB VISUALIZATION_SOURCES
            "${CMAKE_SOURCE_DIR}/src/visualization/*.cpp"
        )
        file(GLOB VISUALIZATION_HEADERS
            "${CMAKE_SOURCE_DIR}/include/visualization/*.h"
        )

        ADD_LIBRARY(Visualization STATIC ${VISUALIZATION_SOURCES} ${VISUALIZATION_HEADERS})
	TARGET_LINK_LIBRARIES(Visualization Robot Leg Kinematic)
        INSTALL(TARGETS Visualization RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${VISUALIZATION_HEADERS} DESTINATION include/visualization/)
endif(BUILD_CONTROLLER AND BUILD_CONTROLLER_VISUALIZATION)

###############################################################################
#
# CONTROLLER DEMO executables
#
###############################################################################

if(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_BASIC)

        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(demo ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(demo Leg Board Robot Visualization)
        INSTALL(TARGETS demo RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_BASIC)


###############################################################################
#
# CONTROLLER BOARD DEMO executables
#
###############################################################################

if(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_BOARD)

        SET(DEMO_BOARD_SOURCES ./demoBoard.cpp)
        ADD_EXECUTABLE(demoBoard ${DEMO_BOARD_SOURCES})
        TARGET_LINK_LIBRARIES(demoBoard Leg Board Robot Visualization)
        INSTALL(TARGETS demoBoard RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_BOARD_FILES} DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_BOARD)

###############################################################################
#
# CONTROLLER BOARD DEMO executables
#
###############################################################################

if(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_LEG)

        SET(DEMO_LEG_SOURCES ./demoLeg.cpp)
        ADD_EXECUTABLE(demoLeg ${DEMO_LEG_SOURCES})
        TARGET_LINK_LIBRARIES(demoLeg Leg Board Robot Visualization)
        INSTALL(TARGETS demoLeg RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_LEG_FILES} DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_LEG)


###############################################################################
#
# CONTROLLER KINEMATIC DEMO executables
#
###############################################################################

if(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_KINEMATIC)

        SET(DEMO_KINEMATIC_SOURCES ./demoKinematic.cpp)
        ADD_EXECUTABLE(demoKinematic ${DEMO_KINEMATIC_SOURCES})
        TARGET_LINK_LIBRARIES(demoKinematic Leg Board Robot Visualization)
        INSTALL(TARGETS demoKinematic RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_KINEMATIC_FILES} DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_KINEMATIC)

###############################################################################
#
# CONTROLLER ROBOT DEMO executables
#
###############################################################################

if(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_ROBOT)

        SET(DEMO_ROBOT_SOURCES ./demoRobot.cpp)
        ADD_EXECUTABLE(demoRobot ${DEMO_ROBOT_SOURCES})
        TARGET_LINK_LIBRARIES(demoRobot Leg Board Robot Visualization)
        INSTALL(TARGETS demoRobot RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_ROBOT_FILES} DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_ROBOT)

###############################################################################
#
# CONTROLLER VISUALIZATION DEMO executables
#
###############################################################################

if(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_VISUALIZATION)

        SET(DEMO_VISUALIZATION_SOURCES ./demoVisualization.cpp)
        ADD_EXECUTABLE(demoVisualization ${DEMO_VISUALIZATION_SOURCES})
        TARGET_LINK_LIBRARIES(demoVisualization Leg Board Robot Visualization ${X11_LIBRARIES} ${ZLIB_LIBRARIES} "/usr/lib/x86_64-linux-gnu/libIrrlicht.so")
        INSTALL(TARGETS demoVisualization RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_VISUALIZATION_FILES} DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO AND BUILD_CONTROLLER_DEMO_VISUALIZATION)

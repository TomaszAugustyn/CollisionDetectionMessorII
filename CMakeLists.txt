# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Example Project configuration
PROJECT(COLLISIONDETECTIONMESSORII)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")

# core libraries
option(BUILD_COLLISIONDETECTIONMESSORII "Build PUT SLAM libraries" YES)
option(BUILD_COLLISIONDETECTIONMESSORII_DEFS "Build TUTORIAL Defs library" YES)
mark_as_advanced(BUILD_COLLISIONDETECTIONMESSORII_DEFS)
option(BUILD_COLLISIONDETECTIONMESSORII_COLDET "Build COLLISIONDETECTIONMESSORII COLDET library" YES)
mark_as_advanced(BUILD_COLLISIONDETECTIONMESSORII_COLDET)
# option(BUILD_COLLISIONDETECTIONMESSORII_VISUALIZER "Build TUTORIAL visualizer library" YES)
# mark_as_advanced(BUILD_TUTORIAL_VISUALIZER)

# demos
option(BUILD_COLLISIONDETECTIONMESSORII_DEMO "Build COLLISIONDETECTIONMESSORII demonstration programs and libraries" YES)
option(BUILD_COLLISIONDETECTIONMESSORII_DEMO_BASIC "Build COLLISIONDETECTIONMESSORII test" YES)
mark_as_advanced(BUILD_COLLISIONDETECTIONMESSORII_DEMO_BASIC)
# option(BUILD_TUTORIAL_DEMO_GRABBER "Build grabber test" YES)
# mark_as_advanced(BUILD_TUTORIAL_DEMO_GRABBER)

#additional dependencies
# Boost
#FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
# OpenCV
# FIND_PACKAGE(OpenCV REQUIRED)
# include_directories( ${OpenCV_INCLUDE_DIRS} )
# PCL
# FIND_PACKAGE(PCL REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

#Eigen
#add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

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
# TUTORIAL tinyXML2 library
#
###############################################################################

#add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)
add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)

###############################################################################
#
# TUTORIAL defs library
#
###############################################################################

if(BUILD_COLLISIONDETECTIONMESSORII AND BUILD_COLLISIONDETECTIONMESSORII_DEFS)
        file(GLOB DEFS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Defs/defs.h"
        )
        ADD_LIBRARY(COLLISIONDETECTIONMESSORII_DEFS STATIC ${DEFS_HEADERS})
        set_target_properties(COLLISIONDETECTIONMESSORII_DEFS PROPERTIES LINKER_LANGUAGE CXX)
# 	INSTALL(TARGETS PUTSLAMDefs RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/Defs/)

endif(BUILD_COLLISIONDETECTIONMESSORII AND BUILD_COLLISIONDETECTIONMESSORII_DEFS)

###############################################################################
#
# TUTORIAL Grabber library
#
###############################################################################

 if(BUILD_COLLISIONDETECTIONMESSORII AND BUILD_COLLISIONDETECTIONMESSORII_COLDET)
       file(GLOB COLDET_SOURCES
            "${CMAKE_SOURCE_DIR}/src/CollisionDetection/*.cpp"
        )
        file(GLOB COLDET_HEADERS
            "${CMAKE_SOURCE_DIR}/include/CollisionDetection/*.h"
        )

        ADD_LIBRARY(Coldet STATIC ${COLDET_SOURCES} ${COLDET_HEADERS})
        TARGET_LINK_LIBRARIES(Coldet)
#        INSTALL(TARGETS Coldet RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${COLDET_HEADERS} GRABBERKINECT_HEADERS DESTINATION include/TUTORIAL/Grabber/)
 endif(BUILD_COLLISIONDETECTIONMESSORII AND BUILD_COLLISIONDETECTIONMESSORII_COLDET)
###############################################################################
#
# TUTORIAL Visualizer
#
###############################################################################

# if(BUILD_TUTORIAL AND BUILD_TUTORIAL_VISUALIZER)
#        file(GLOB VISUALIZER_SOURCES
#            "${CMAKE_SOURCE_DIR}/src/Visualizer/*.cpp"
#        )
#        file(GLOB VISUALIZER_HEADERS
#            "${CMAKE_SOURCE_DIR}/include/Visualizer/*.h"
#        )
#        ADD_LIBRARY(Visualizer STATIC ${VISUALIZER_SOURCES} ${VISUALIZER_HEADERS})
#		TARGET_LINK_LIBRARIES(Visualizer)
#        INSTALL(TARGETS Visualizer RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
#        INSTALL(FILES ${VISUALIZER_HEADERS} DESTINATION include/putslam/Visualizer/)
# endif(BUILD_TUTORIAL AND BUILD_TUTORIAL_VISUALIZER)

###############################################################################
#
# TUTORIAL DEMO executables
#
###############################################################################

if(BUILD_COLLISIONDETECTIONMESSORII_DEMO AND BUILD_COLLISIONDETECTIONMESSORII_DEMO_BASIC)

        SET(DEMO_FILES
                ${PROJECT_ROOT}/resources/configGlobal.xml
        )

        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(Demo tinyxml2)
        INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

endif(BUILD_COLLISIONDETECTIONMESSORII_DEMO AND BUILD_COLLISIONDETECTIONMESSORII_DEMO_BASIC)

###############################################################################
#
# TUTORIAL DEMO Grabber executables
#
###############################################################################

# if(BUILD_TUTORIAL_DEMO AND BUILD_TUTORIAL_DEMO_GRABBER)
#
#        SET(DEMO_FILES
#                ${PROJECT_ROOT}/resources/configGlobal.xml
#        )
#        #SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
#        SET(DEMO_SOURCES ./demo_grabber.cpp)
#        ADD_EXECUTABLE(Demo_grabber ${DEMO_SOURCES})
#        TARGET_LINK_LIBRARIES(Demo_grabber tinyxml2 Grabber Visualizer)
#        INSTALL(TARGETS Demo_grabber RUNTIME DESTINATION bin)
#        INSTALL(FILES ${DEMO_FILES} DESTINATION bin)

# endif(BUILD_TUTORIAL_DEMO AND BUILD_TUTORIAL_DEMO_GRABBER)

###################################################################
## project name and used language
PROJECT(Sheet02MORMS14 CXX)

###################################################################
## check environment and require functionality
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.2)


###################################################################
## external modules - load a convenience functionality
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake/modules" )
INCLUDE( copySrc )

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/src )

###################################################################
## Add executables

## Excerice 1
ADD_EXECUTABLE( ASSIGNMENT1
  src/main_2.1.cc
)
SET_TARGET_PROPERTIES( ASSIGNMENT1 PROPERTIES
  OUTPUT_NAME "assigment_1" )

## Exercice 2
ADD_EXECUTABLE( ASSIGNMENT2
  src/main_2.2.cc
)
SET_TARGET_PROPERTIES( ASSIGNMENT2 PROPERTIES
  OUTPUT_NAME "assigment_2" )

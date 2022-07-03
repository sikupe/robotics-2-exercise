####################################################################################################
#### BASIC CONFIGURATION
####################################################################################################
CMAKE_MINIMUM_REQUIRED ( VERSION 2.6 )

PROJECT( "muscod_sample_application" CXX )

LIST( APPEND CMAKE_MODULE_PATH
	${PROJECT_SOURCE_DIR}/CMake
	${PROJECT_SOURCE_DIR}/CMake/Modules
)

INCLUDE( SimOptDefaults )

####################################################################################################
#### GET USED PACKAGES
####################################################################################################
FIND_PACKAGE( MUSCOD REQUIRED )  # find and include "MUSCODConfig.cmake"
IF( MUSCOD_FOUND )
	INCLUDE( ${MUSCOD_USE_FILE} )
ENDIF( MUSCOD_FOUND )


####################################################################################################
#### BUILD LIBRARY OF THE OPTIMAL CONTROL PROBLEM
####################################################################################################

# NOTE: Here you need to add all source files that are required for your
#       optimal control problem.

ADD_LIBRARY ( bvp1
        SRC/bvp1.cc
	)

ADD_LIBRARY ( bvp2
        SRC/bvp2.cc
	)


ADD_LIBRARY ( bvp3
        SRC/bvp3.cc
	)

ADD_LIBRARY ( bvp4
        SRC/bvp4.cc
	)
# COMMANDS TO BUILD DIRECTORY TREE #############################################

EXECUTE_PROCESS(COMMAND mkdir -p ./RES)
EXECUTE_PROCESS(COMMAND ln -s -f ${PROJECT_SOURCE_DIR}/SRC)
EXECUTE_PROCESS(COMMAND ln -s -f ${PROJECT_SOURCE_DIR}/DAT)
EXECUTE_PROCESS(COMMAND ln -s -f ${PROJECT_SOURCE_DIR}/default.plot)

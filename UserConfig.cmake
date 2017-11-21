# This file defines the user-specific settings.

set(MA27LIB_DIR   "/homes/bcdandurand/comp_infrastructure/ma27-1.0.0/lib")
set(CPLEX_LIB_DIR "/homes/bcdandurand/comp_infrastructure/CPLEX/cplex/lib/x86-64_linux/static_pic")
set(CPLEX_INC_DIR "/homes/bcdandurand/comp_infrastructure/CPLEX/cplex/include/ilcplex")
set(DEPEND_DIR    $ENV{PWD})

# Please change OFF to ON once the settings are provided.
set(USER_SETTINGS ON)

if(NOT ${USER_SETTINGS})
	message(FATAL_ERROR "Please complete the user-specific settings in UserConfig.cmake")
endif()

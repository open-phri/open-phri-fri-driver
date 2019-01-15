CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(open-phri-fri-driver)

PID_Package(
			AUTHOR 		    Benjamin Navarro
			INSTITUTION	    LIRMM
			MAIL		    navarro@lirmm.fr
			ADDRESS 	    git@github.com:BenjaminNavarro/open-phri-fri-driver.git
            PUBLIC_ADDRESS  https://github.com/BenjaminNavarro/open-phri-fri-driver.git
			YEAR 		    2018
			LICENSE 	    GNULGPL
			DESCRIPTION     "An OpenPHRI driver for the Kuka LWR4 robot, based on the Fast Research Interface."
			VERSION			0.1.0
)

PID_Dependency(open-phri VERSION 1.0)
PID_Dependency(api-driver-fri VERSION 1.3)

build_PID_Package()

CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(open-phri-fri-driver)

declare_PID_Package(
			AUTHOR 		    Benjamin Navarro
			INSTITUTION	    LIRMM
			MAIL		    navarro@lirmm.fr
			ADDRESS 	    git@github.com:BenjaminNavarro/open-phri-fri-driver.git
            PUBLIC_ADDRESS  https://github.com/BenjaminNavarro/open-phri-fri-driver.git
			YEAR 		    2018
			LICENSE 	    GNULGPL
			DESCRIPTION     "An OpenPHRI driver for the Kuka LWR4 robot, based on the Fast Research Interface."
)

set_PID_Package_Version(0 1)

#now finding packages
declare_PID_Package_Dependency(PACKAGE open-phri NATIVE 0.8)
declare_PID_Package_Dependency(PACKAGE api-driver-fri NATIVE VERSION 1.3)

build_PID_Package()

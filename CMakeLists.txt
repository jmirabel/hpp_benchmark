#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp_benchmark
# hpp_benchmark is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_benchmark is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_benchmark  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp_benchmark)
SET(PROJECT_DESCRIPTION
  "Tutorial for humanoid path planner platform."
)
SET(PROJECT_URL "")

FINDPYTHON()

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

ADD_DOC_DEPENDENCY("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-gepetto-viewer")

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  urdf/lydia.urdf
  urdf/obstacle.urdf
  urdf/obstacle_medium.urdf
  urdf/obstacle_easy.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES srdf/lydia.srdf 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install (FILES
  src/hpp/corbaserver/lydia/robot.py
  src/hpp/corbaserver/lydia/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/lydia)
SETUP_PROJECT_FINALIZE()

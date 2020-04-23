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

SET(PROJECT_NAME hpp_benchmark)
SET(PROJECT_DESCRIPTION "Benchmarks for humanoid path planner platform.")

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

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

install(FILES
  future/baxter-manipulation-boxes/script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/baxter-manipulation-boxes)

install(FILES
  future/construction-set/script.py
  future/construction-set/setup.py
  future/construction-set/state_name.py
  future/construction-set/visibility_prm.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/construction-set)

install(FILES
  future/hrp2-on-the-ground/script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/hrp2-on-the-ground)

install(FILES
  future/pr2-in-iai-kitchen/script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pr2-in-iai-kitchen)

install(FILES
  future/pr2-manipulation-kitchen/script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pr2-manipulation-kitchen)

install(FILES
  future/pr2-manipulation-two-hand/script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pr2-manipulation-two-hand)

install(FILES
  future/romeo-placard/script.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/romeo-placard)

ADD_SUBDIRECTORY(src)

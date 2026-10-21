cmake_minimum_required(VERSION 3.0.2)
project(mocap_distance_viz)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

set(NODE_SCRIPTS
  scripts/distance_viz_node
  scripts/fake_mocap_publisher
)

# Melodic (Python 2): use copies of the scripts with a 'python' shebang.
if(PYTHON_VERSION_MAJOR EQUAL 2)
  set(_py2_scripts)
  foreach(_script ${NODE_SCRIPTS})
    get_filename_component(_name ${_script} NAME)
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/${_script} _content)
    string(REGEX REPLACE "^#!/usr/bin/env python3" "#!/usr/bin/env python" _content "${_content}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/py2/${_name} "${_content}")
    list(APPEND _py2_scripts ${CMAKE_CURRENT_BINARY_DIR}/py2/${_name})
  endforeach()
  # Melodic creates no devel-space wrappers, so place the copies there as well.
  file(COPY ${_py2_scripts}
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  set(NODE_SCRIPTS ${_py2_scripts})
endif()

catkin_install_python(PROGRAMS ${NODE_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_tracker.py)
  catkin_add_nosetests(test/test_terminal.py)
  find_package(rostest REQUIRED)
  add_rostest(test/distance_viz.test)
endif()

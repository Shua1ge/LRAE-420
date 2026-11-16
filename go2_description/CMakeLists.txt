cmake_minimum_required(VERSION 3.0.2)
project(go2_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config dae launch meshes models scripts xacro
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

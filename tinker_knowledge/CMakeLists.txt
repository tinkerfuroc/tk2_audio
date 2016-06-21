cmake_minimum_required(VERSION 2.8.3)
project(tinker_knowledge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

if(NOT EXISTS model/en-us/)
    add_custom_target(checkout_model ALL
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/script/checkout_model.sh ${CMAKE_CURRENT_SOURCE_DIR}/model 
    )
endif(NOT EXISTS model/en-us/)

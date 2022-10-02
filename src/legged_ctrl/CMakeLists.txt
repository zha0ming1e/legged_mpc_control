cmake_minimum_required(VERSION 3.10.1)
project(legged_ctrl)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++14")

#check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# Find Eigen3
find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)


# Find catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        unitree_legged_msgs
        ocs2_legged_robot
        qpoases_catkin
        tf
        )
include_directories(
        ${catkin_INCLUDE_DIRS}
)


# add pinocchio and ocs2
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        DEPENDS
        pinocchio
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)
link_directories(
        ${pinocchio_LIBRARY_DIRS}
)


# Find OSQP
include_directories(
        /usr/local/include/osqp
)
find_package(OsqpEigen REQUIRED)

# Find Library for julia 
add_subdirectory(src/mpc_ctrl/ci_mpc/lib/EmbeddedLciMpc.jl)
include_directories(${EmbeddedLciMpc_INCLUDE_DIRS})
link_directories(${EmbeddedLciMpc_LIB_DIRS})

# Find unitree hardware library
include_directories(
        $ENV{UNITREE_LEGGED_SDK_PATH}/include
)
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

# Find altro
# find_package(AltroCpp REQUIRED)


# test programs
#add_executable(test_altro src/test/test_altro.cpp)
#target_link_libraries(test_altro
#  PRIVATE
#  altro::altro
#)



# test ocs2 
add_library(wbc_ctrl_lib
        src/wbc_ctrl/HoQp.cpp
        src/wbc_ctrl/wbc.cpp
        )


target_link_libraries(wbc_ctrl_lib
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )
target_compile_options(wbc_ctrl_lib PUBLIC ${OCS2_CXX_FLAGS})   


add_library(utils_lib
        src/utils/LeggedIKSolver.cpp
        src/utils/Utils.cpp
        src/LeggedState.cpp
        src/utils/A1Kinematics.cpp
        )
target_link_libraries(utils_lib
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )
     
target_compile_options(utils_lib PUBLIC ${OCS2_CXX_FLAGS})   


add_library(interfaces_lib
        src/interfaces/BaseInterface.cpp
        src/interfaces/GazeboInterface.cpp
        src/LeggedState.cpp
        src/utils/Utils.cpp
        src/estimation/BasicEKF.cpp
        src/wbc_ctrl/wbc.cpp
        src/wbc_ctrl/HoQp.cpp
        src/utils/LeggedIKSolver.cpp
        src/utils/A1Kinematics.cpp
        )
target_link_libraries(interfaces_lib
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(interfaces_lib PUBLIC ${OCS2_CXX_FLAGS})   


add_library(ci_mpc_lib
        src/mpc_ctrl/LeggedMPC.cpp
        src/mpc_ctrl/ci_mpc/LciMpc.cpp
        )
target_link_libraries(ci_mpc_lib
        ${catkin_LIBRARIES} 
        OsqpEigen::OsqpEigen 
        juliaCpp_lib
        )

add_library(convex_mpc_lib
src/mpc_ctrl/LeggedMPC.cpp
src/mpc_ctrl/convex_mpc/ConvexMpc.cpp
src/mpc_ctrl/convex_mpc/ConvexQPSolver.cpp
src/utils/LeggedContactFSM.cpp
)

target_link_libraries(convex_mpc_lib
${catkin_LIBRARIES} 
OsqpEigen::OsqpEigen 
)


############# main prorgam ######################
add_executable(main
        src/main.cpp
)

target_link_libraries(main
        utils_lib
        interfaces_lib
        ci_mpc_lib
        convex_mpc_lib
        ${pinocchio_LIBRARIES}
)


add_executable(wbc_test
        src/test/test_wbc.cpp
        )

target_link_libraries(wbc_test
        wbc_ctrl_lib
        utils_lib
        ${pinocchio_LIBRARIES}
        )

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(ho_qp_test
            src/test/ho_qp_test.cpp
            )

    target_link_libraries(ho_qp_test
            wbc_ctrl_lib
            ${pinocchio_LIBRARIES}
            gtest_main
            )

endif (CATKIN_ENABLE_TESTING)
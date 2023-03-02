cmake_minimum_required(VERSION 2.8.3)
project(server_pose_graph)

add_definitions(-w)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fopenmp")


#set(cv_bridge_DIR /usr/local/cv_bridge341/share/cv_bridge/cmake)





find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    pcl_ros
    cv_bridge
    camera_model
    agent_msg
    open_chisel
    roslib
)


set(OpenCV_DIR "/usr/local/share/OpenCV") 
find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

find_package(Sophus REQUIRED)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(
	${catkin_INCLUDE_DIRS} 
	${CERES_INCLUDE_DIRS}  
	${EIGEN3_INCLUDE_DIR}
	${Sophus_INCLUDE_DIR}
    )

#Pangolin
FIND_PACKAGE( Pangolin )
INCLUDE_DIRECTORIES( ${Pangolin_INCLUDE_DIRS} )

#Octomap
FIND_PACKAGE(octomap REQUIRED)
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS})

#Cuda
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    #    -std=c++11 -O3 -gencode arch=compute_30,code=sm_30  # T440s
    #    -std=c++11 -O3 -gencode arch=compute_53,code=sm_53  # tx1
    -std=c++11 -O3 -gencode arch=compute_75,code=sm_75  # gtx 980 Ti / 1080
    )



catkin_package()


cuda_add_executable(server_pose_graph


    src/chisel/camera/Intrinsics.cpp
    src/chisel/camera/PinholeCamera.cpp
    src/chisel/geometry/AABB.cpp
    src/chisel/geometry/Frustum.cpp
    src/chisel/geometry/Plane.cpp
    src/chisel/geometry/Raycast.cpp
    src/chisel/io/PLY.cpp
    src/chisel/marching_cubes/MarchingCubes.cpp
    src/chisel/mesh/Mesh.cpp
    src/chisel/pointcloud/PointCloud.cpp
    src/chisel/Chisel.cpp
    src/chisel/Chunk.cpp
    src/chisel/ChunkManager.cpp
    src/chisel/ColorVoxel.cpp
    src/chisel/DistVoxel.cpp
    src/chisel/ProjectionIntegrator.cpp
    src/chisel/chisel_wrapper.cpp

    src/collaborative_server_node.cpp
    src/collaborative_server_system.cpp
    src/parameters.cpp
    src/server_pose_graph.cpp
    src/optimizer/smooth_euler_optimizer.cpp
    src/utility/server_utility.cpp
    src/utility/server_visualization.cpp
    src/server_camera.cpp
    src/server_brief_extractor.cpp
    src/server_keyframe.cpp
    src/server_global_map.cpp



    ThirdParty/DBoW/BowVector.cpp
    ThirdParty/DBoW/FBrief.cpp
    ThirdParty/DBoW/FeatureVector.cpp
    ThirdParty/DBoW/QueryResults.cpp
    ThirdParty/DBoW/ScoringObject.cpp
    ThirdParty/DUtils/Random.cpp
    ThirdParty/DUtils/Timestamp.cpp
    ThirdParty/DVision/BRIEF.cpp
    ThirdParty/VocabularyBinary.cpp
    )


target_link_libraries(server_pose_graph 
	${catkin_LIBRARIES} 
	${OpenCV_LIBS} 
	${CERES_LIBRARIES}
	${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${CUDA_LIBRARIES}
    ) 



catkin_install_python(PROGRAMS
  scripts/recover_submap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

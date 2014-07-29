cmake_minimum_required(VERSION 2.8.3)
project(pointCloudStatistic)

find_package( PCL REQUIRED)
include_directories(include
			              ${PCL_INCLUDE_DIRS}
			              /home/jp/devel/src/perception3d/include
)

link_directories(${PCL_LIBRARY_DIRS}
			           /home/jp/devel/devel/lib/liblperception.so
)
add_definitions(${PCL_DEFINITIONS})

add_executable(pointCloudStatistic
		           src/mainProgram.cpp)
		           
target_link_libraries(pointCloudStatistic 
			                ${PCL_LIBRARIES}
			                /home/jp/devel/devel/lib/liblperception.so
)

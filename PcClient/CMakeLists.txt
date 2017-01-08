cmake_minimum_required(VERSION 2.8)
project( pcUsbTool )

find_package( OpenCV REQUIRED )

include_directories(
)

add_executable( streamer
    video.cpp
)

target_link_libraries( streamer ${OpenCV_LIBS} -lpthread -lm )

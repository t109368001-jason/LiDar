cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#避免建置路徑與程式碼路徑相同
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

#清除舊的檔案
set(cmake_generated 
)
foreach(file
        ${CMAKE_BINARY_DIR}/CMakeCache.txt
        ${CMAKE_BINARY_DIR}/cmake_install.cmake  
        ${CMAKE_BINARY_DIR}/Makefile
        ${CMAKE_BINARY_DIR}/CMakeFiles
        ${CMAKE_BINARY_DIR}/bin
    )
    if (EXISTS ${file})
        file(REMOVE_RECURSE ${file})
    endif()

endforeach(file)

#設定變數
set(PROJECT_NAME pcl_viewer)
set(SOURCE_FILE main.cpp)
set(SOURCE_FILE main.cpp CACHE FILEPATH "SOURCE_FILE")  #cmake -D SOURCE_FILE=xxx ..
set(BUILD_EXAMPLE OFF)
set(BUILD_EXAMPLE OFF CACHE BOOL "BUILD_EXAMPLE")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_BINARY_DIR}/bin
    CACHE PATH
    "Output directory for applications"
    )

project(${PROJECT_NAME})

find_package(PCL 1.2 REQUIRED)  #尋找PCL函式庫

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-std=c++17)   #boost需要版以上

add_executable(${PROJECT_NAME} ${SOURCE_FILE})          #設定執行檔名稱及對應程式碼
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}) #連結函式庫

if(BUILD_EXAMPLE)   #設定其他範例執行檔
    add_executable(pcd_viewer ${CMAKE_BINARY_DIR}/../example/pcd_viewer.cpp)
    target_link_libraries(pcd_viewer ${PCL_LIBRARIES})
    add_executable(pcd_poisson_viewer ${CMAKE_BINARY_DIR}/../example/pcd_poisson_viewer.cpp)
    target_link_libraries(pcd_poisson_viewer ${PCL_LIBRARIES})
    add_executable(pcap_viewer ${CMAKE_BINARY_DIR}/../example/pcap_viewer.cpp)
    target_link_libraries(pcap_viewer ${PCL_LIBRARIES})
    add_executable(pcd_XYZR_XYZRGB ${CMAKE_BINARY_DIR}/../example/pcd_XYZR_XYZRGB.cpp)
    target_link_libraries(pcd_XYZR_XYZRGB ${PCL_LIBRARIES})
endif()
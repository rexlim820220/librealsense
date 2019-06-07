#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesMyexample)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
  #myexample
    add_executable(rs-myexample rs-myexample.cpp ../example.hpp ../../third-party/imgui/imgui.cpp ../../third-party/imgui/imgui_draw.cpp ../../third-party/imgui/imgui_impl_glfw.cpp /usr/local/include/opencv4/opencv2/opencv.hpp)
    target_link_libraries(rs-myexample ${DEPENDENCIES})
    include_directories(${OpenCV_INCLUDE_DIRS} rs-myexample ../ ../../third-party/tclap/include ../../third-party/imgui)
    set_target_properties (rs-myexample PROPERTIES FOLDER Examples)
    install(TARGETS rs-myexample RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# Find Packages
find_package( PCL 1.9 REQUIRED )

if( PCL_FOUND )
  include_directories( ${PCL_INCLUDE_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )
  link_directories( ${PCL_LIBRARY_DIRS} )
  target_link_libraries( rs-myexample ${PCL_LIBRARIES} )
endif()

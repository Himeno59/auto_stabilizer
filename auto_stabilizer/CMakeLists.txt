cmake_minimum_required(VERSION 3.0.2)
project(auto_stabilizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

option(USE_SANITIZER "Enable address sanitizer" OFF)
message(STATUS "USE_SANITIZER: ${USE_SANITIZER}")
if(USE_SANITIZER)
  message(STATUS "Enable Sanitizer for basketball_motion_controller")
  set(SANITIZE_FLAGS "-fsanitize=address")
  set(CMAKE_CXX_FLAGS "${SANITIZE_FLAGS}")
  set(CMAKE_C_FLAGS "${SANITIZE_FLAGS}")
  set(CMAKE_CXX_FLAGS_DEBUG "${SANITIZE_FLAGS}")
  set(CMAKE_C_FLAGS_DEBUG "${SANITIZE_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${SANITIZE_FLAGS}")
  set(CMAKE_C_FLAGS_RELEASE "${SANITIZE_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${SANITIZE_FLAGS}")
  set(CMAKE_C_FLAGS_RELWITHDEBINFO "${SANITIZE_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
  )

# catkin_python_setup()
# initialize rtmbuild (generate_messages is called)
rtmbuild2_init(rtmbuild2)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES AutoStabilizerServiceSkel AutoStabilizerServiceStub
  CATKIN_DEPENDS rtmbuild2
#  DEPENDS system_lib
)

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

###########
## Build ##
###########

add_subdirectory(rtc)

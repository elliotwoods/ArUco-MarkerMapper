# CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
LINK_LIBRARIES(${PROJECT_NAME})

ADD_EXECUTABLE(mapper_from_video mapper_from_video.cpp)
ADD_EXECUTABLE(mapper_from_images mapper_from_images.cpp)
#ADD_EXECUTABLE(mapper_live mapper_live.cpp)
ADD_EXECUTABLE(mapper_viewer mapper_viewer.cpp sgl.h  sglviewer.h)
#ADD_EXECUTABLE(mapper_dummygame mapper_dummygame.cpp sgl.h  sglviewer.h)

INSTALL(TARGETS mapper_from_images mapper_from_video   mapper_viewer     RUNTIME DESTINATION bin)

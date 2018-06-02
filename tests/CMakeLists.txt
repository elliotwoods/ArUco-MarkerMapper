# CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/src/utils)
LINK_LIBRARIES(${PROJECT_NAME})

#Export matches to be computed by other SfM methods

#ADD_EXECUTABLE(test_errorfunction test_errorfunction.cpp)
ADD_EXECUTABLE(test_manu test_manu.cpp   )
ADD_EXECUTABLE(log2pcd log2pcd.cpp   )


ADD_EXECUTABLE(create_markers create_markers.cpp)
ADD_EXECUTABLE(graphsolver  graphsolver.cpp)


ADD_EXECUTABLE(test_optimization test_optimization.cpp   )
ADD_EXECUTABLE(markermapper2pcd markermapper2pcd.cpp   )
ADD_EXECUTABLE(addvideodata2mapper addvideodata2mapper.cpp   )
ADD_EXECUTABLE(leica2pcd leica2pcd.cpp   )
ADD_EXECUTABLE(markermapper2map markermapper2map.cpp   )

find_package( cvba )
IF(cvba_FOUND)
ADD_EXECUTABLE(markermapper2cvba markermapper2cvba.cpp   )
target_link_libraries(markermapper2cvba ${cvba_LIBS})
ENDIF()

#find_package( depthmaps )
#IF(depthmaps_FOUND)
#ADD_EXECUTABLE(pcdsubsample pcdsubsample.cpp   )
#ADD_EXECUTABLE(extractpatch extractpatch.cpp   )
#target_link_libraries(extractpatch ${depthmaps_LIBS})
#target_link_libraries(pcdsubsample ${depthmaps_LIBS})
#ENDIF()


ADD_EXECUTABLE(preprocessvideo preprocessvideo.cpp   )
ADD_EXECUTABLE(preprocessvideo_fromimages preprocessvideo_fromimages.cpp   )
ADD_EXECUTABLE(ppv_process ppv_process.cpp   )
ADD_EXECUTABLE(ppv_tracking ppv_tracking.cpp   )
add_executable(test_ambig test_ambig.cpp)
#ADD_EXECUTABLE(test_video test_video.cpp   )

ADD_EXECUTABLE(test_keypoints test_keypoints.cpp   )

ADD_EXECUTABLE(test_met test_met.cpp   )

ADD_EXECUTABLE(test_utils3d test_utils3d.cpp utils3d.cpp utils3d.h  )



##USEFUL
ADD_EXECUTABLE(leicamarkers2pcd leicamarkers2pcd.cpp   )

ADD_EXECUTABLE(compare_logs compare_logs.cpp)
ADD_EXECUTABLE(compare2board compare2board.cpp)
ADD_EXECUTABLE(leica_error leica_error.cpp   )
ADD_EXECUTABLE(leica_error_frommap leica_error_frommap.cpp   )

ADD_EXECUTABLE(posegraphfromfile  posegraphfromfile.cpp   )

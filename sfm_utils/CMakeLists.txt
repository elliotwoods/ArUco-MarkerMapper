# CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
LINK_LIBRARIES(${PROJECT_NAME})

#Export matches to be computed by other SfM methods
ADD_EXECUTABLE(bundler_step1_exportmatches bundler_step1_exportmatches.cpp   )
ADD_EXECUTABLE(openmvg_step1_exportmatches openmvg_step1_exportmatches.cpp   )
ADD_EXECUTABLE(openmvg_step2_generate_pcd openmvg_step2_generate_pcd.cpp   )
ADD_EXECUTABLE(vsfm_step1_exportmatches vsfm_step1_exportmatches.cpp   )
ADD_EXECUTABLE(vsfm_step2_generate_ply_log vsfm_step2_generate_ply_log.cpp   )
  # evaluate_vsfm_file.cpp

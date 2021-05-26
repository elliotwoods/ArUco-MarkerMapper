INCLUDE_DIRECTORIES(.)

FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.cpp")

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs  ${srcs_base} )


SET(mappers_hdr mappers/globalgraph_markermapper.h mappers/graph.h     mappers/stgmatrix.h   mappers/posegraphoptimizer.h)
SET(mappers_src mappers/globalgraph_markermapper.cpp   mappers/posegraphoptimizer.cpp)

SET(utils_hdr utils/kdtree.h  utils/nanoflann.h    utils/se3.h utils/utils3d.h  utils/mm_omp.h)
SET(utils_src utils/utils3d.cpp)


SET (optimizers_hdr optimizers/ippe.h     optimizers/sparselevmarq.h optimizers/fullsceneoptimizer.h   )
SET (optimizers_src optimizers/ippe.cpp     optimizers/fullsceneoptimizer.cpp )

SET(hdrs ${hdrs} ${optimizers_hdr} ${mappers_hdr} ${utils_hdr})
SET(srcs ${srcs} ${optimizers_src} ${mappers_src} ${utils_src})

SET(LIBNAME ${EXTRALIBNAME}marker_mapper)
ADD_LIBRARY(${LIBNAME} ${srcs} ${hdrs})

IF(NOT COMPILE_IN_ANDROID)
SET_TARGET_PROPERTIES(${LIBNAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL MARKERMAPPER_DSO_EXPORTS
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_SOVERSION}
	CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
        OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)

TARGET_LINK_LIBRARIES(${LIBNAME} ${EXTRALIBNAME}aruco ${OpenCV_LIBS})

INSTALL(TARGETS ${LIBNAME}
	RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
	LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory

IF(MARKERMAPPER_DEVINSTALL)

INSTALL(FILES ${hdrs_base}
        DESTINATION include/marker_mapper
        COMPONENT main)

INSTALL(FILES ${utils_hdr}
        DESTINATION include/marker_mapper/utils
        COMPONENT main)
INSTALL(FILES ${optimizers_hdr}
        DESTINATION include/marker_mapper/optimizers
        COMPONENT main)
    ENDIF()
ELSE()
    MESSAGE(STATUS  "MARKERMAPPER COMPILATION")
    target_link_libraries( # Specifies the target library.
                           marker_mapper aruco lib_opencv

                           # Links the target library to the log library
                           # included in the NDK.
                           ${log-lib} )
ENDIF()

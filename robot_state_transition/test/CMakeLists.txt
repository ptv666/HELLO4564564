include_directories("${INCLUDE_DIR}")

add_executable(testGLPK testGLPK.cpp)
target_link_libraries(testGLPK ${RST_LIBRARY_NAME})

add_executable(test_Bretl test_Bretl.cpp)
target_link_libraries(test_Bretl ${RST_LIBRARY_NAME})

add_executable(test_CWC_region test_CWC_region.cpp)
target_link_libraries(test_CWC_region ${RST_LIBRARY_NAME})
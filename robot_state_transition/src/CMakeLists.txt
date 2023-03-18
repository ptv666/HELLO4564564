include_directories("${INCLUDE_DIR}")

#########--------RST库的构建--------#########
#RST源文件
set(${RST_LIBRARY_NAME}_SOURCES
		${RST_HEADERS}
		${SRC_DIR}/solve_LP_GLPK.cpp
		${SRC_DIR}/Bretl.cpp
)
add_library(${RST_LIBRARY_NAME} SHARED ${${RST_LIBRARY_NAME}_SOURCES})		#构建RST动态链接库
target_link_libraries(${RST_LIBRARY_NAME} libglpk.so)	                    #链接外部库
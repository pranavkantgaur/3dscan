
cmake_minimum_required(VERSION 2.8)

SET( SOURCES get_homog calib_camera )

FOREACH(src ${SOURCES})
	ADD_EXECUTABLE(${src} ${src}.cxx)
	TARGET_LINK_LIBRARIES(${src} ${OpenCV_LIBS} )
ENDFOREACH(src)


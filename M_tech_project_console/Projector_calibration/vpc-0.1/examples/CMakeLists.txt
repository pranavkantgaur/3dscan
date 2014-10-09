
SET( SOURCES main_direct main_sampling main_autocalib )

FOREACH(src ${SOURCES})
	ADD_EXECUTABLE(${src} ${src}.cxx)
	TARGET_LINK_LIBRARIES(${src} ${OpenCV_LIBS} vpc)
ENDFOREACH(src)



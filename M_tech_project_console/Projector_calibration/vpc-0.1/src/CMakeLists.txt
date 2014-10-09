
ADD_LIBRARY( vpc SHARED vpc_base.cxx htrki.cxx vpc_bundle.cxx sampling.cxx direct.cxx autocalib.cxx)
TARGET_LINK_LIBRARIES( vpc ${OpenCV_LIBS})

FILE(GLOB HDRS "${CMAKE_SOURCE_DIR}/include/*.h")
INSTALL(FILES ${HDRS} DESTINATION include/vpc)
INSTALL(TARGETS vpc 
	LIBRARY DESTINATION lib
	ARCHIVE  DESTINATION lib)

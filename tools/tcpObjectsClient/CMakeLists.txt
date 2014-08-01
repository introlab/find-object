### Qt Gui stuff ###
SET(headers_ui 
	TcpClient.h
)
#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###

SET(SRC_FILES
    TcpClient.cpp
    main.cpp
    ${moc_srcs} 
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
	${OpenCV_LIBS} 
	${QT_LIBRARIES} 
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "example" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(tcpObjectsClient ${SRC_FILES})
TARGET_LINK_LIBRARIES(tcpObjectsClient ${LIBRARIES})

SET_TARGET_PROPERTIES( tcpObjectsClient 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-tcpObjectsClient)

INSTALL(TARGETS tcpObjectsClient
        RUNTIME DESTINATION bin COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
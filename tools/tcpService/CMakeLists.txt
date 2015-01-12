
SET(SRC_FILES
    main.cpp 
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
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
ADD_EXECUTABLE(tcpService ${SRC_FILES})
TARGET_LINK_LIBRARIES(tcpService find_object ${LIBRARIES})

SET_TARGET_PROPERTIES( tcpService 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-tcpService)
  
INSTALL(TARGETS tcpService
        RUNTIME DESTINATION bin COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)


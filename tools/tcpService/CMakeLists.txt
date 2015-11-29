
SET(SRC_FILES
    main.cpp 
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

IF("${FINDOBJECT_QT_VERSION}" STREQUAL "4")
    INCLUDE(${QT_USE_FILE})
ENDIF()

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
IF("${FINDOBJECT_QT_VERSION}" STREQUAL "5")
    QT5_USE_MODULES(tcpService Widgets Core Gui Network PrintSupport)
ENDIF()

SET_TARGET_PROPERTIES( tcpService 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-tcpService)
  
INSTALL(TARGETS tcpService
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)


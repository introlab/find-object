

SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
)

SET(LIBRARIES
	${OpenCV_LIBS} 
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "similarity" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(similarity ${SRC_FILES})
TARGET_LINK_LIBRARIES(similarity ${LIBRARIES})

SET_TARGET_PROPERTIES( similarity 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-similarity)

INSTALL(TARGETS similarity
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)

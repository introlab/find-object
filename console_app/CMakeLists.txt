

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

# Add binary called "console" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(console ${SRC_FILES})
TARGET_LINK_LIBRARIES(console ${LIBRARIES})

SET_TARGET_PROPERTIES( console 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-console)


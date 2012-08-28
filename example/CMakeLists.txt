### Qt Gui stuff ###
SET(headers_ui 
	../src/ObjWidget.h
	../src/Camera.h
)
#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###

SET(SRC_FILES
    main.cpp
    ../src/ObjWidget.cpp
    ../src/KeypointItem.cpp
    ../src/QtOpenCV.cpp
    ../src/Settings.cpp
    ../src/Camera.cpp
    ../src/utilite/UDirectory.cpp
    ../src/utilite/UFile.cpp
    ${moc_srcs} 
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/../src
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
ADD_EXECUTABLE(example ${SRC_FILES})
TARGET_LINK_LIBRARIES(example ${LIBRARIES})

SET_TARGET_PROPERTIES( example 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-example)


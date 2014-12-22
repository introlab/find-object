

### Qt Gui stuff ###
SET(headers_ui 
	../include/${PROJECT_PREFIX}/MainWindow.h
	../include/${PROJECT_PREFIX}/FindObject.h
	../include/${PROJECT_PREFIX}/Camera.h
	../include/${PROJECT_PREFIX}/TcpServer.h
	../include/${PROJECT_PREFIX}/ObjWidget.h
	./AddObjectDialog.h
	./CameraTcpServer.h
	./ParametersToolBox.h
	./AboutDialog.h
	./RectItem.h
	./ImageDropWidget.h
	./rtabmap/PdfPlot.h
	./utilite/UPlot.h
)

SET(uis
    ./ui/mainWindow.ui
    ./ui/addObjectDialog.ui
    ./ui/aboutDialog.ui
)

SET(qrc 
    ./resources.qrc
)

# generate rules for building source files from the resources
QT4_ADD_RESOURCES(srcs_qrc ${qrc})

#Generate .h files from the .ui files
QT4_WRAP_UI(moc_uis ${uis})

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###

SET(SRC_FILES 
	./MainWindow.cpp
	./AddObjectDialog.cpp
	./KeypointItem.cpp
	./RectItem.cpp
	./QtOpenCV.cpp
	./Camera.cpp
	./CameraTcpServer.cpp
	./ParametersToolBox.cpp
	./Settings.cpp
	./ObjWidget.cpp
	./ImageDropWidget.cpp
	./FindObject.cpp
	./AboutDialog.cpp
	./TcpServer.cpp
	./Vocabulary.cpp
	./JsonWriter.cpp
	./utilite/ULogger.cpp
	./utilite/UPlot.cpp
	./utilite/UDirectory.cpp
	./utilite/UFile.cpp
	./utilite/UConversion.cpp
	./rtabmap/PdfPlot.cpp
	./json/jsoncpp.cpp
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
    ${QT_LIBRARIES} 
    ${OpenCV_LIBS} 
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# create a library from the source files
ADD_LIBRARY(find_object ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(find_object ${LIBRARIES})

INSTALL(TARGETS find_object
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT runtime
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel)
        
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

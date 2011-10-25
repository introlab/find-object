

### Qt Gui stuff ###
SET(headers_ui 
	./MainWindow.h
	./AddObjectDialog.h
	./Object.h
	./Camera.h
	./ParametersToolBox.h
)

SET(uis
    ./ui/mainWindow.ui
    ./ui/addObjectDialog.ui
)

#SET(qrc 
#    ./GuiLib.qrc
#)

# generate rules for building source files from the resources
#QT4_ADD_RESOURCES(srcs_qrc ${qrc})

#Generate .h files from the .ui files
QT4_WRAP_UI(moc_uis ${uis})

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###




SET(SRC_FILES 
	./MainWindow.cpp
	./AddObjectDialog.cpp
	./KeypointItem.cpp
	./qtipl.cpp
	./main.cpp
	./Camera.cpp
	./ParametersToolBox.cpp
	./Settings.cpp
	./Object.cpp
	${moc_srcs} 
	${moc_uis} 
	#${srcs_qrc}
)

SET(INCLUDE_DIRS
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

# create an executable file
ADD_EXECUTABLE(find_object ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(find_object ${LIBRARIES})




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
IF(CATKIN_BUILD)
   SET(headers_ui  
      ${headers_ui}
      ./ros/CameraROS.h
      ./ros/FindObjectROS.h
   )
ENDIF(CATKIN_BUILD)

SET(uis
   ./ui/mainWindow.ui
   ./ui/addObjectDialog.ui
   ./ui/aboutDialog.ui
)

SET(qrc 
   ./resources.qrc
)

IF(QT4_FOUND)
    # generate rules for building source files from the resources
    QT4_ADD_RESOURCES(srcs_qrc ${qrc})

    #Generate .h files from the .ui files
    QT4_WRAP_UI(moc_uis ${uis})

    #This will generate moc_* for Qt
    QT4_WRAP_CPP(moc_srcs ${headers_ui})
    ### Qt Gui stuff  end###
ELSE()
    QT5_ADD_RESOURCES(srcs_qrc ${qrc})
    QT5_WRAP_UI(moc_uis ${uis})
    QT5_WRAP_CPP(moc_srcs ${headers_ui})
ENDIF()

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
IF(CATKIN_BUILD)
   SET(SRC_FILES 
      ${SRC_FILES}
      ./ros/CameraROS.cpp
      ./ros/FindObjectROS.cpp
   )
ENDIF(CATKIN_BUILD)



SET(INCLUDE_DIRS
   ${CMAKE_CURRENT_SOURCE_DIR}/../include
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${OpenCV_INCLUDE_DIRS}
   ${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)
IF(CATKIN_BUILD)
   SET(INCLUDE_DIRS
      ${INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
   )
ENDIF(CATKIN_BUILD)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
   ${QT_LIBRARIES} 
   ${OpenCV_LIBS} 
)
IF(CATKIN_BUILD)
   SET(LIBRARIES
      ${LIBRARIES}
      ${catkin_LIBRARIES}
   )
ENDIF(CATKIN_BUILD)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# create a library from the source files
ADD_LIBRARY(find_object ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(find_object ${LIBRARIES})
IF(Qt5_FOUND)
    QT5_USE_MODULES(find_object Widgets Core Gui Network PrintSupport)
ENDIF(Qt5_FOUND)
IF(CATKIN_BUILD)
   set_target_properties(find_object PROPERTIES  OUTPUT_NAME find_object_2d)
   add_dependencies(find_object ${${PROJECT_NAME}_EXPORTED_TARGETS})
ENDIF(CATKIN_BUILD)

IF(NOT CATKIN_BUILD)
   INSTALL(TARGETS find_object
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
      
   install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
ELSE()
   add_executable(find_object_2d ros/find_object_2d_node.cpp)
   target_link_libraries(find_object_2d find_object ${LIBRARIES})

   add_executable(print_objects_detected ros/print_objects_detected_node.cpp)
   target_link_libraries(print_objects_detected ${LIBRARIES})
   add_dependencies(print_objects_detected ${${PROJECT_NAME}_EXPORTED_TARGETS})

   add_executable(tf_example ros/tf_example_node.cpp)
   target_link_libraries(tf_example ${LIBRARIES})
   add_dependencies(tf_example ${${PROJECT_NAME}_EXPORTED_TARGETS})
   
   IF(Qt5_FOUND)
      QT5_USE_MODULES(find_object_2d Widgets Core Gui Network PrintSupport)
      QT5_USE_MODULES(print_objects_detected Widgets Core Gui Network PrintSupport)
      QT5_USE_MODULES(tf_example Widgets Core Gui Network PrintSupport)
   ENDIF(Qt5_FOUND)

   ## Mark executables and/or libraries for installation
   install(TARGETS 
      find_object
      find_object_2d 
      print_objects_detected 
      tf_example 
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
ENDIF()

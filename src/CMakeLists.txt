

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
ADD_EXECUTABLE(find_object WIN32 ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(find_object ${LIBRARIES})

IF(WIN32)
  SET_TARGET_PROPERTIES(find_object PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME})
ENDIF(WIN32)


#---------------------------
# Installation stuff
#--------------------------- 
INSTALL(TARGETS find_object
        RUNTIME DESTINATION bin COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
            
SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX}")
IF(WIN32)
  SET(plugin_dest_dir bin)
  SET(qtconf_dest_dir bin)

  # Install needed Qt plugins by copying directories from the qt installation
  # One can cull what gets copied by using 'REGEX "..." EXCLUDE'
  # Exclude debug libraries
  INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" 
          DESTINATION ${plugin_dest_dir}/plugins 
          COMPONENT runtime 
          REGEX ".*d4.dll" EXCLUDE
          REGEX ".*d4.a" EXCLUDE)
  
  # install a qt.conf file
  # this inserts some cmake code into the install script to write the file
  SET(QT_CONF_FILE [Paths]\nPlugins=plugins)
  INSTALL(CODE "
    file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"${QT_CONF_FILE}\")
    " COMPONENT runtime)

  # directories to look for dependencies
  SET(DIRS ${QT_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/bin)

  # Now the work of copying dependencies into the bundle/package
  # The quotes are escaped and variables to use at install time have their $ escaped
  # An alternative is the do a configure_file() on a script and use install(SCRIPT  ...).
  # Note that the image plugins depend on QtSvg and QtXml, and it got those copied
  # over.
  # To find dependencies, cmake use "otool" on Apple and "dumpbin" on Windows (make sure you have one of them).
  install(CODE "
   file(GLOB_RECURSE QTPLUGINS \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
    include(\"BundleUtilities\")
    fixup_bundle(\"${APPS}\" \"\${QTPLUGINS}\" \"${DIRS}\")
  " COMPONENT runtime)
ENDIF(WIN32)


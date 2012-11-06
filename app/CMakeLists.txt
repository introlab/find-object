

### Qt Gui stuff ###
SET(headers_ui 
	../src/MainWindow.h
	../src/AddObjectDialog.h
	../src/ObjWidget.h
	../src/Camera.h
	../src/ParametersToolBox.h
	../src/AboutDialog.h
	../src/utilite/UPlot.h
	../src/rtabmap/PdfPlot.h
)

SET(uis
    ../src/ui/mainWindow.ui
    ../src/ui/addObjectDialog.ui
    ../src/ui/aboutDialog.ui
)

SET(qrc 
    ../src/resources.qrc
)

# generate rules for building source files from the resources
QT4_ADD_RESOURCES(srcs_qrc ${qrc})

#Generate .h files from the .ui files
QT4_WRAP_UI(moc_uis ${uis})

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###


SET(SRC_FILES 
	./main.cpp
	../src/MainWindow.cpp
	../src/AddObjectDialog.cpp
	../src/KeypointItem.cpp
	../src/QtOpenCV.cpp
	../src/Camera.cpp
	../src/ParametersToolBox.cpp
	../src/Settings.cpp
	../src/ObjWidget.cpp
	../src/AboutDialog.cpp
	../src/utilite/UPlot.cpp
	../src/utilite/UDirectory.cpp
	../src/utilite/UFile.cpp
	../src/rtabmap/PdfPlot.cpp
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../src
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

# For Apple set the icns file containing icons
IF(APPLE AND BUILD_AS_BUNDLE)
  # set how it shows up in the Info.plist file
  SET(MACOSX_BUNDLE_ICON_FILE ${PROJECT_NAME}.icns) 
  # set where in the bundle to put the icns file
  SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  # include the icns file in the target
  SET(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.icns)
ENDIF(APPLE AND BUILD_AS_BUNDLE)

# Add exe icon resource
IF(WIN32)
  IF( MINGW )
    # resource compilation for MinGW
    ADD_CUSTOM_COMMAND( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/AppIco.o
                        COMMAND windres.exe -I${CMAKE_CURRENT_SOURCE_DIR} -i${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.rc 
                             -o ${CMAKE_CURRENT_BINARY_DIR}/AppIco.o )
    SET(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_BINARY_DIR}/AppIco.o)
  ELSE( MINGW )
   # SET(SRC_FILES ${SRC_FILES} ${PROJECT_NAME}.rc) # Error on visual studio 2008
  ENDIF( MINGW )
ENDIF(WIN32)

# create an executable file
IF(APPLE AND BUILD_AS_BUNDLE)
  ADD_EXECUTABLE(find_object MACOSX_BUNDLE ${SRC_FILES})
ELSEIF(MINGW)
  ADD_EXECUTABLE(find_object WIN32 ${SRC_FILES})
ELSE()
  ADD_EXECUTABLE(find_object ${SRC_FILES})
ENDIF()
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(find_object ${LIBRARIES})

IF(APPLE AND BUILD_AS_BUNDLE)
  SET_TARGET_PROPERTIES(find_object PROPERTIES
    OUTPUT_NAME ${CMAKE_BUNDLE_NAME})
ELSEIF(WIN32)
  SET_TARGET_PROPERTIES(find_object PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME})
ELSE()
  SET_TARGET_PROPERTIES(find_object PROPERTIES
    OUTPUT_NAME ${PROJECT_PREFIX})
ENDIF()

#---------------------------
# Installation stuff
#--------------------------- 
INSTALL(TARGETS find_object
        RUNTIME DESTINATION bin COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
        
#---------------------------
# Dependencies packaging
#---------------------------
IF(APPLE AND BUILD_AS_BUNDLE)
  INSTALL(CODE "execute_process(COMMAND ln -s \"../MacOS/${CMAKE_BUNDLE_NAME}\" ${PROJECT_NAME}
        WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin)")
ENDIF(APPLE AND BUILD_AS_BUNDLE)

SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX}")
IF((APPLE AND BUILD_AS_BUNDLE) OR WIN32)
  SET(plugin_dest_dir bin)
  SET(qtconf_dest_dir bin)
  IF(APPLE)
    SET(plugin_dest_dir MacOS)
    SET(qtconf_dest_dir Resources)
    SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/MacOS/${CMAKE_BUNDLE_NAME}")
  ENDIF(APPLE)
  
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
  IF(APPLE)
    SET(QT_CONF_FILE [Paths]\nPlugins=MacOS/plugins)
  ENDIF(APPLE)
  INSTALL(CODE "
    file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"${QT_CONF_FILE}\")
    " COMPONENT runtime)

  # directories to look for dependencies
  SET(DIRS ${QT_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/bin)
  IF(APPLE)
    SET(DIRS ${DIRS} /usr/local)
  ENDIF(APPLE)

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
ENDIF((APPLE AND BUILD_AS_BUNDLE) OR WIN32)


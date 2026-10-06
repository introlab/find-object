
SET(headers_ui 
	TcpServerPool.h
)

IF(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs ${headers_ui})
ELSEIF(Qt5_FOUND)
    QT5_WRAP_CPP(moc_srcs ${headers_ui})
ELSE()
    QT6_WRAP_CPP(moc_srcs ${headers_ui})
ENDIF()

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
    ${QT_LIBRARIES} 
    ${OpenCV_LIBS} 
)

if(Tcmalloc_FOUND)
    SET(LIBRARIES
        ${LIBARIES} 
        ${Tcmalloc_LIBRARIES}
    )
endif(Tcmalloc_FOUND)


#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

SET(SRC_FILES main.cpp ${moc_srcs} )

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
    SET(SRC_FILES ${SRC_FILES} ${PROJECT_NAME}.rc) # Error on visual studio 2008
  ENDIF( MINGW )
ENDIF(WIN32)

# create an executable file
IF(APPLE AND BUILD_AS_BUNDLE)
  ADD_EXECUTABLE(find_object_app MACOSX_BUNDLE ${SRC_FILES})
ELSEIF(MINGW)
  ADD_EXECUTABLE(find_object_app WIN32 ${SRC_FILES})
ELSE()
  ADD_EXECUTABLE(find_object_app ${SRC_FILES})
ENDIF()
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(find_object_app find_object ${LIBRARIES})

IF(APPLE AND BUILD_AS_BUNDLE)
  SET_TARGET_PROPERTIES(find_object_app PROPERTIES
    OUTPUT_NAME ${CMAKE_BUNDLE_NAME})
ELSEIF(WIN32)
  SET_TARGET_PROPERTIES(find_object_app PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME})
ELSE()
  SET_TARGET_PROPERTIES(find_object_app PROPERTIES
    OUTPUT_NAME ${PROJECT_PREFIX})
ENDIF()

#---------------------------
# Installation stuff
#--------------------------- 
INSTALL(TARGETS find_object_app
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
        
#---------------------------
# Dependencies packaging
#---------------------------
IF(APPLE AND BUILD_AS_BUNDLE)
  INSTALL(CODE "execute_process(COMMAND ln -s \"../MacOS/${CMAKE_BUNDLE_NAME}\" ${PROJECT_NAME}
        WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin)")
ENDIF(APPLE AND BUILD_AS_BUNDLE)

IF((APPLE AND BUILD_AS_BUNDLE) OR WIN32)
  SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX}")
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
  IF(QT_PLUGINS_DIR)
    INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats"
          DESTINATION ${plugin_dest_dir}/plugins 
          COMPONENT runtime 
          REGEX ".*d4.dll" EXCLUDE
          REGEX ".*d4.a" EXCLUDE)
  ELSEIF(Qt6_FOUND)
    # Find the plugins directory from the platform plugin, then install the
    # plugin types the app needs. fixup_bundle() below deploys the libraries.
    IF(WIN32)
      SET(qt_platform_plugin Qt6::QWindowsIntegrationPlugin)
    ELSE()
      SET(qt_platform_plugin Qt6::QCocoaIntegrationPlugin)
    ENDIF()
    IF(TARGET ${qt_platform_plugin})
      get_target_property(qt_plugins_dir ${qt_platform_plugin} LOCATION)
      get_filename_component(qt_plugins_dir "${qt_plugins_dir}" DIRECTORY) # .../plugins/platforms
      get_filename_component(qt_plugins_dir "${qt_plugins_dir}" DIRECTORY) # .../plugins
      foreach(plugin_type platforms styles imageformats iconengines)
        file(GLOB plugin_files "${qt_plugins_dir}/${plugin_type}/*${CMAKE_SHARED_LIBRARY_SUFFIX}")
        SET(plugin_real_files "")
        foreach(plugin_file ${plugin_files})
          # Homebrew plugins are symlinks into the Cellar, install the real files
          get_filename_component(plugin_file "${plugin_file}" REALPATH)
          LIST(APPEND plugin_real_files "${plugin_file}")
        endforeach()
        IF(plugin_real_files)
          INSTALL(FILES ${plugin_real_files}
            DESTINATION ${plugin_dest_dir}/plugins/${plugin_type}
            COMPONENT runtime)
        ENDIF()
      endforeach()
    ELSE()
      MESSAGE(WARNING "${qt_platform_plugin} target not found: Qt plugins will be missing from the package")
    ENDIF()
  ELSE()
	#Qt5
	foreach(plugin ${Qt5Gui_PLUGINS})
		get_target_property(plugin_loc ${plugin} LOCATION)
		get_filename_component(plugin_dir ${plugin_loc} DIRECTORY)
		string(REPLACE "plugins" ";" loc_list ${plugin_dir})
		list(GET loc_list 1 plugin_type)
	    #MESSAGE(STATUS "Qt5 plugin \"${plugin_loc}\" installed in \"${plugin_dest_dir}/plugins${plugin_type}\"")
		INSTALL(FILES ${plugin_loc}
          DESTINATION ${plugin_dest_dir}/plugins${plugin_type}
          COMPONENT runtime)
	endforeach()
  ENDIF()
  
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
    SET(DIRS ${DIRS} /usr/local /usr/local/lib /opt/homebrew /opt/homebrew/lib /opt/homebrew/lib/gcc/current)
  ENDIF(APPLE)

  # Now the work of copying dependencies into the bundle/package
  # The quotes are escaped and variables to use at install time have their $ escaped
  # An alternative is the do a configure_file() on a script and use install(SCRIPT  ...).
  # Note that the image plugins depend on QtSvg and QtXml, and it got those copied
  # over.
  # To find dependencies, cmake use "otool" on Apple and "dumpbin" on Windows (make sure you have one of them).
  install(CODE "
   file(GLOB_RECURSE QTPLUGINS \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
    # Homebrew libraries are read-only, make the copies writable so they can be fixed up
    set(BU_CHMOD_BUNDLE_ITEMS ON)
    include(\"BundleUtilities\")
    fixup_bundle(\"${APPS}\" \"\${QTPLUGINS}\" \"${DIRS}\")
  " COMPONENT runtime)
ENDIF((APPLE AND BUILD_AS_BUNDLE) OR WIN32)


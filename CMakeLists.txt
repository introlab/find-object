cmake_minimum_required(VERSION 2.8.5)

# Detect if it is called by catkin
IF(CATKIN_TOPLEVEL OR CATKIN_BUILD_BINARY_PACKAGE OR CATKIN_SKIP_TESTING OR CATKIN_ENABLE_TESTING OR CATKIN_DEVEL_PREFIX)
  SET(CATKIN_BUILD TRUE)
ENDIF(CATKIN_TOPLEVEL OR CATKIN_BUILD_BINARY_PACKAGE OR CATKIN_SKIP_TESTING OR CATKIN_ENABLE_TESTING OR CATKIN_DEVEL_PREFIX)

MESSAGE(STATUS "CATKIN_BUILD=${CATKIN_BUILD}")

IF(NOT CATKIN_BUILD)
   #Standalone build
   PROJECT( Find-Object )
ELSE()
   #ROS catkin build
   PROJECT( find_object_2d )
ENDIF()

# Catkin doesn't support multiarch library path, 
# fix to "lib" if not set by user.
IF(NOT DEFINED CMAKE_INSTALL_LIBDIR)
   set(CMAKE_INSTALL_LIBDIR "lib")
ENDIF(NOT DEFINED CMAKE_INSTALL_LIBDIR)

INCLUDE(GNUInstallDirs)

#######################
# VERSION
#######################
SET(PROJECT_VERSION "0.6.2")
SET(PROJECT_PREFIX find_object)

STRING(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
LIST(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
LIST(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
LIST(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)

ADD_DEFINITIONS(-DPROJECT_PREFIX="${PROJECT_PREFIX}")
ADD_DEFINITIONS(-DPROJECT_VERSION="${PROJECT_VERSION}")
ADD_DEFINITIONS(-DPROJECT_NAME="${PROJECT_NAME}")

####### DEPENDENCIES #######
FIND_PACKAGE(OpenCV REQUIRED) # tested on 2.3.1

# look for Qt5 before Qt4
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Network QUIET)
IF(NOT Qt5_FOUND)
   FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
ENDIF(NOT Qt5_FOUND)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS) # To avoid conflicts with boost signals used in ROS

SET(NONFREE 0)
IF(OPENCV_NONFREE_FOUND OR OPENCV_XFEATURES2D_FOUND)
   SET(NONFREE 1)
ENDIF(OPENCV_NONFREE_FOUND OR OPENCV_XFEATURES2D_FOUND)

CONFIGURE_FILE(Version.h.in ${PROJECT_SOURCE_DIR}/include/${PROJECT_PREFIX}/Version.h)

IF(NOT CATKIN_BUILD)
   #Standalone build
   IF(WIN32 AND NOT MINGW)
       ADD_DEFINITIONS("-wd4251")
   ELSE ()
      ADD_DEFINITIONS( "-Wall" )
   ENDIF()
   #ADD_DEFINITIONS("-DUNICODE") # to test with UNICODE projects

   ####### COMPILATION PARAMS #######
   # In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
   IF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")
       IF("${CMAKE_BUILD_TYPE}" STREQUAL "")
      set(CMAKE_BUILD_TYPE Release)
       ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "")
   ENDIF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")

   SET(CMAKE_DEBUG_POSTFIX "d")

   ####### Build libraries as shared or static #######
   OPTION( BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON )

   ####### OUTPUT DIR #######
   SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
   SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
   SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

   ####### INSTALL DIR #######
   # Offer the user the choice of overriding the installation directories
   set(INSTALL_INCLUDE_DIR include/${PROJECT_PREFIX}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} CACHE PATH
     "Installation directory for header files")
   if(WIN32 AND NOT CYGWIN)
     set(DEF_INSTALL_CMAKE_DIR CMake)
   else()
     set(DEF_INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_PREFIX}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
   endif()
   set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
     "Installation directory for CMake files")
   

   ####### OSX BUNDLE CMAKE_INSTALL_PREFIX #######
   IF(APPLE)
      OPTION(BUILD_AS_BUNDLE "Set to ON to build as bundle (DragNDrop)" OFF)
   ENDIF(APPLE)
   IF(APPLE AND BUILD_AS_BUNDLE) 
       # Required when packaging, and set CMAKE_INSTALL_PREFIX to "/".
       SET(CPACK_SET_DESTDIR TRUE) 
     
       SET(CMAKE_BUNDLE_NAME
         "${PROJECT_NAME}")
       SET(CMAKE_BUNDLE_LOCATION "/")
       
       # make sure CMAKE_INSTALL_PREFIX ends in /
       SET(CMAKE_INSTALL_PREFIX 
         "/${CMAKE_BUNDLE_NAME}.app/Contents")
   ENDIF(APPLE AND BUILD_AS_BUNDLE)

   ####### SOURCES (Projects) #######
   ADD_SUBDIRECTORY( src )
   ADD_SUBDIRECTORY( app )
   ADD_SUBDIRECTORY( tools )
   IF(NONFREE)
   ADD_SUBDIRECTORY( example )
   ENDIF(NONFREE)

   #######################
   # Uninstall target, for "make uninstall"
   #######################
   CONFIGURE_FILE(
     "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
     "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
     IMMEDIATE @ONLY)

   ADD_CUSTOM_TARGET(uninstall
     "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

   #######################
   # Setup FindObjectConfig.cmake
   #######################
   # Create the FindObjectConfig.cmake and FindObjectConfigVersion files
   file(RELATIVE_PATH REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}")
   file(RELATIVE_PATH REL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
      
   # ... for the build tree
   set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
   set(CONF_LIB_DIR "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
   configure_file(FindObjectConfig.cmake.in
     "${PROJECT_BINARY_DIR}/FindObjectConfig.cmake" @ONLY)
     
   # ... for the install tree
   set(CONF_INCLUDE_DIRS "\${FindObject_CMAKE_DIR}/${REL_INCLUDE_DIR}")
   set(CONF_LIB_DIR "\${FindObject_CMAKE_DIR}/${REL_LIB_DIR}")
   configure_file(FindObjectConfig.cmake.in
     "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FindObjectConfig.cmake" @ONLY)
     
   # ... for both
   configure_file(FindObjectConfigVersion.cmake.in
     "${PROJECT_BINARY_DIR}/FindObjectConfigVersion.cmake" @ONLY)

   # Install the FindObjectConfig.cmake and FindObjectConfigVersion.cmake
   install(FILES
     "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FindObjectConfig.cmake"
     "${PROJECT_BINARY_DIR}/FindObjectConfigVersion.cmake"
     DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT devel)
   ####

   #######################
   # CPACK (Packaging)
   #######################
   INCLUDE(InstallRequiredSystemLibraries)

   SET(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
   SET(CPACK_PACKAGE_VENDOR "${PROJECT_NAME} project")
   SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Find-Object")
   SET(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
   SET(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
   SET(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
   SET(CPACK_PACKAGE_CONTACT "matlabbe@gmail.com")

   set(CPACK_SOURCE_IGNORE_FILES 
     "\\\\.svn/" 
     "${PROJECT_SOURCE_DIR}/build/[a-zA-Z0-9_]+" 
     "~$" 
     "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_PREFIX}"
     "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_NAME}"
     "\\\\.DS_Store"
   )

   IF(WIN32)
     SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
     IF(CMAKE_CL_64) 
       SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64") 
     ELSE() 
       SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES") 
     ENDIF()
     SET(CPACK_GENERATOR "ZIP;NSIS")
     SET(CPACK_SOURCE_GENERATOR "ZIP")
     SET(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME}")
     SET(ICON_PATH "${PROJECT_SOURCE_DIR}/app/${PROJECT_NAME}.ico")
     SET(CPACK_NSIS_MUI_ICON ${ICON_PATH})
     SET(CPACK_NSIS_MUI_UNIICON ${ICON_PATH})
     SET(CPACK_NSIS_DISPLAY_NAME "${PROJECT_NAME}")
     SET(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
     # Set the icon used for the Windows "Add or Remove Programs" tool.
     SET(CPACK_NSIS_INSTALLED_ICON_NAME bin\\\\${PROJECT_NAME}.exe)
     SET(CPACK_PACKAGE_EXECUTABLES "${PROJECT_NAME}" "${PROJECT_NAME}" ${CPACK_PACKAGE_EXECUTABLES}) 
     SET(CPACK_CREATE_DESKTOP_LINKS "${PROJECT_NAME}" ${CPACK_CREATE_DESKTOP_LINKS}) 
     SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME}")
   ELSEIF(APPLE)
     IF(BUILD_AS_BUNDLE)
       # On APPLE and if BUILD_AS_BUNDLE=ON, the project is created as a bundle 
       # over the main app (see ./src).Here we package only this bundle. Note 
       # that we set CMAKE_INSTALL_PREFIX to "/" when packaging to DragNDrop...
       SET(CPACK_GENERATOR "DragNDrop")
     ELSE()
       SET(CPACK_GENERATOR "PackageMaker;TBZ2")
     ENDIF()
     
     SET(CPACK_SOURCE_GENERATOR "TBZ2")

     SET(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}/app/${PROJECT_NAME}.icns")
   ELSE()
     SET(CPACK_SOURCE_GENERATOR "ZIP")
   ENDIF()

   INCLUDE(CPack)

   #######################
   # OUTPUT INFO
   #######################
   MESSAGE(STATUS "--------------------------------------------")
   MESSAGE(STATUS "Info :")
   MESSAGE(STATUS "  CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
   MESSAGE(STATUS "  CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
   MESSAGE(STATUS "  PROJECT_VERSION = ${PROJECT_VERSION}")
 
   IF(OpenCV_VERSION_MAJOR EQUAL 2)
     IF(OPENCV_NONFREE_FOUND)
       MESSAGE(STATUS "  With OpenCV 2 nonfree module (SIFT/SURF) = YES")
     ELSE()
       MESSAGE(STATUS "  With OpenCV 2 nonfree module (SIFT/SURF) = NO (not found)")
     ENDIF()
   ELSE()
     IF(OPENCV_XFEATURES2D_FOUND)
       MESSAGE(STATUS "  With OpenCV 3 xfeatures2d module (SIFT/SURF/BRIEF/FREAK) = YES")
     ELSE()
       MESSAGE(STATUS "  With OpenCV 3 xfeatures2d module (SIFT/SURF/BRIEF/FREAK) = NO (not found)")
     ENDIF()
   ENDIF()

   IF(QT4_FOUND)
      MESSAGE(STATUS "  With Qt4 = YES")
   ELSEIF(Qt5_FOUND)
      MESSAGE(STATUS "  With Qt5 = YES")
   ELSE()
      MESSAGE(STATUS "  With Qt = NO (Qt not found)")
   ENDIF()

   IF(APPLE)
   MESSAGE(STATUS "  BUILD_AS_BUNDLE = ${BUILD_AS_BUNDLE}")
   ENDIF(APPLE)
   MESSAGE(STATUS "--------------------------------------------")

ELSE()
   #ROS Catkin build

   ## Find catkin macros and libraries
   ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
   ## is used, also find other catkin packages
   find_package(catkin REQUIRED COMPONENTS 
           cv_bridge roscpp rospy sensor_msgs std_msgs image_transport genmsg message_filters tf
   )

   ## Generate messages in the 'msg' folder
   add_message_files(
      FILES
      ObjectsStamped.msg
   )

   ## Generate added messages and services with any dependencies listed here
   generate_messages(
      DEPENDENCIES
      std_msgs
      sensor_msgs
   )

   ###################################
   ## catkin specific configuration ##
   ###################################
   ## The catkin_package macro generates cmake config files for your package
   ## Declare things to be passed to dependent projects
   ## INCLUDE_DIRS: uncomment this if you package contains header files
   ## LIBRARIES: libraries you create in this project that dependent projects also need
   ## CATKIN_DEPENDS: catkin_packages dependent projects also need
   ## DEPENDS: system dependencies of this project that dependent projects also need
   catkin_package(
     CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf
   )

   ###########
   ## Build ##
   ###########
   ADD_SUBDIRECTORY( src )

   #############
   ## Install ##
   #############
   ## Mark other files for installation (e.g. launch and bag files, etc.)
   install(FILES
      launch/find_object_2d_gui.launch
      launch/find_object_2d.launch
      launch/find_object_3d.launch
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   )
ENDIF()


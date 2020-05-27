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

####### local cmake modules #######
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# Policy CMP0043 introduced in cmake version 3.0 IGNORES the use of COMPILE_DEFINITIONS suffixed variables, e.g. COMPILE_DEFINITIONS_DEBUG
# Set to OLD behavior until minimum cmake version >= 2.8.10 (version that COMPILE_DEFINITIONS can be set by generator expressions instead)
if (POLICY CMP0043)
   cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0043)
# Policy CMP0042 introduced in cmake version 3.0 enables the use of @rpath in an install name via MACOSX_RPATH by default
# Set to OLD behavior so that all versions use the same behavior, or until minimum cmake version >= 2.8.12 (version where @rpath is available)
if (POLICY CMP0042)
   cmake_policy(SET CMP0042 OLD)
endif (POLICY CMP0042)

#######################
# VERSION
#######################
SET(PROJECT_VERSION "0.6.3")
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

# check if version status is "-dev" (SIFT compatibility issue between 4.3.0 vs 4.3.0-dev)
FIND_FILE(OpenCV_VERSION_HPP opencv2/core/version.hpp
  PATHS ${OpenCV_INCLUDE_DIRS}
  NO_DEFAULT_PATH)
FILE(READ ${OpenCV_VERSION_HPP} TMPTXT)
STRING(FIND "${TMPTXT}" "-dev" matchres)
IF(NOT ${matchres} EQUAL -1)
  add_definitions(-DOPENCV_DEV)
ENDIF(NOT ${matchres} EQUAL -1)

# For SuperPoint
SET(TORCH 0)
FIND_PACKAGE(Torch QUIET)
IF(TORCH_FOUND)
    MESSAGE(STATUS "Found Torch: ${TORCH_INCLUDE_DIRS}")
    SET(TORCH 1)
ENDIF(TORCH_FOUND)

IF(OpenCV_VERSION_MAJOR EQUAL 4)
  IF(NOT MSVC)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    IF(COMPILER_SUPPORTS_CXX11)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    ELSEIF(COMPILER_SUPPORTS_CXX0X)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    ELSE()
      message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    ENDIF()
  ENDIF(NOT MSVC)
ENDIF(OpenCV_VERSION_MAJOR EQUAL 4)

# look for Qt5 before Qt4
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Network QUIET)
IF(NOT Qt5_FOUND)
   FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
ENDIF(NOT Qt5_FOUND)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS) # To avoid conflicts with boost signals used in ROS

FIND_PACKAGE(Tcmalloc QUIET)

FIND_PACKAGE(ZLIB REQUIRED QUIET)

SET(NONFREE 1)
IF(NOT (OPENCV_NONFREE_FOUND OR OPENCV_XFEATURES2D_FOUND))
   SET(NONFREE 0)
ELSEIF(OpenCV_VERSION VERSION_GREATER "3.4.2")
  FIND_FILE(OpenCV_MODULES_HPP opencv2/opencv_modules.hpp 
    PATHS ${OpenCV_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
  FILE(READ ${OpenCV_MODULES_HPP} TMPTXT)
  STRING(FIND "${TMPTXT}" "#define OPENCV_ENABLE_NONFREE" matchres)
  IF(${matchres} EQUAL -1)
    SET(NONFREE 0)
  ENDIF(${matchres} EQUAL -1)
ENDIF()

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
   IF(NONFREE OR OpenCV_VERSION VERSION_GREATER "4.2.0") # SIFT is free from 4.3.0
   ADD_SUBDIRECTORY( example )
   ENDIF(NONFREE OR OpenCV_VERSION VERSION_GREATER "4.2.0")

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
       MESSAGE(STATUS "  With OpenCV ${OpenCV_VERSION} xfeatures2d module (BRIEF/FREAK/KAZE) = YES")
     ELSE()
       MESSAGE(STATUS "  With OpenCV ${OpenCV_VERSION} xfeatures2d module (BRIEF/FREAK/KAZE) = NO (not found)")
     ENDIF()
     IF(OpenCV_VERSION VERSION_GREATER "4.2.0")
       IF(NONFREE)
         MESSAGE(STATUS "  With OpenCV ${OpenCV_VERSION} nonfree module (SURF) = YES")
       ELSE()
         MESSAGE(STATUS "  With OpenCV ${OpenCV_VERSION} nonfree module (SURF) = NO")
       ENDIF()
     ELSEIF()
       IF(NONFREE)
         MESSAGE(STATUS "  With OpenCV ${OpenCV_VERSION} nonfree module (SIFT/SURF) = YES")
       ELSE()
         MESSAGE(STATUS "  With OpenCV ${OpenCV_VERSION} nonfree module (SIFT/SURF) = NO")
       ENDIF()
     ENDIF()
   ENDIF()

   IF(QT4_FOUND)
      MESSAGE(STATUS "  With Qt4 = YES")
   ELSEIF(Qt5_FOUND)
      MESSAGE(STATUS "  With Qt5 = YES")
   ELSE()
      MESSAGE(STATUS "  With Qt = NO (Qt not found)")
   ENDIF()

   IF(Tcmalloc_FOUND)
      MESSAGE(STATUS "  With tcmalloc = YES")
   ELSE()
      MESSAGE(STATUS "  With tcmalloc = NO (tcmalloc not found)")
   ENDIF(Tcmalloc_FOUND)
   
   IF(TORCH_FOUND)
      MESSAGE(STATUS "  With Torch = YES")
   ELSE()
      MESSAGE(STATUS "  With Torch = NO (libtorch not found)")
   ENDIF(TORCH_FOUND)

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
           cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf message_generation
   )

   ## Generate messages in the 'msg' folder
   add_message_files(
      FILES
      ObjectsStamped.msg
      DetectionInfo.msg
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
     CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf message_runtime
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
      launch/find_object_3d_kinect2.launch
      launch/find_object_3d_zed.launch
      launch/find_object_2d.launch
      launch/find_object_3d.launch
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   )
ENDIF()


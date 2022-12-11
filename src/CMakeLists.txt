

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
      ./ros1/CameraROS.h
      ./ros1/FindObjectROS.h
   )
ENDIF(CATKIN_BUILD)
IF(COLCON_BUILD)
   SET(headers_ui  
      ${headers_ui}
      ./ros2/CameraROS.h
      ./ros2/FindObjectROS.h
   )
ENDIF(COLCON_BUILD)

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
   ./Compression.cpp
   ${moc_srcs} 
   ${moc_uis} 
   ${srcs_qrc}
)
IF(CATKIN_BUILD)
   SET(SRC_FILES 
      ${SRC_FILES}
      ./ros1/CameraROS.cpp
      ./ros1/FindObjectROS.cpp
   )
ENDIF(CATKIN_BUILD)
IF(COLCON_BUILD)
   SET(SRC_FILES 
      ${SRC_FILES}
      ./ros2/CameraROS.cpp
      ./ros2/FindObjectROS.cpp
   )
ENDIF(COLCON_BUILD)

SET(INCLUDE_DIRS
   ${CMAKE_CURRENT_SOURCE_DIR}/../include
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${OpenCV_INCLUDE_DIRS}
   ${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
   ${ZLIB_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
   ${QT_LIBRARIES} 
   ${OpenCV_LIBS} 
   ${ZLIB_LIBRARIES} 
)
IF(CATKIN_BUILD)
   SET(LIBRARIES
      ${LIBRARIES}
      ${catkin_LIBRARIES}
   )
ENDIF(CATKIN_BUILD)
IF(COLCON_BUILD)
   SET(AMENT_LIBRARIES
      rclcpp
      rclcpp_components
      cv_bridge
      sensor_msgs
      std_msgs
      image_transport
      message_filters
      tf2
      tf2_ros
      tf2_geometry_msgs
      geometry_msgs
   )
ENDIF(COLCON_BUILD)


IF(TORCH_FOUND)
	SET(LIBRARIES
		${LIBRARIES}
		${TORCH_LIBRARIES}
	)
	SET(SRC_FILES
		${SRC_FILES}
		superpoint_torch/SuperPoint.cc
	)
	 SET(INCLUDE_DIRS
	    ${TORCH_INCLUDE_DIRS}
	    ${CMAKE_CURRENT_SOURCE_DIR}/superpoint_torch
		${INCLUDE_DIRS}
	)
	ADD_DEFINITIONS("-DWITH_TORCH")
ENDIF(TORCH_FOUND)


#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(CATKIN_BUILD)
   INCLUDE_DIRECTORIES(SYSTEM ${catkin_INCLUDE_DIRS})
ENDIF(CATKIN_BUILD)

# create a library from the source files
ADD_LIBRARY(find_object ${SRC_FILES})
# Linking with Qt libraries
IF(Qt5_FOUND)
    QT5_USE_MODULES(find_object Widgets Core Gui Network PrintSupport)
ENDIF(Qt5_FOUND)

IF(NOT COLCON_BUILD)
   TARGET_LINK_LIBRARIES(find_object ${LIBRARIES})
ENDIF()
IF(CATKIN_BUILD)
   set_target_properties(find_object PROPERTIES  OUTPUT_NAME find_object_2d)
   add_dependencies(find_object ${${PROJECT_NAME}_EXPORTED_TARGETS})
ENDIF(CATKIN_BUILD)
IF(COLCON_BUILD)
   ament_target_dependencies(find_object ${AMENT_LIBRARIES})
   
   if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")
      rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
      target_link_libraries(find_object ${cpp_typesupport_target} ${LIBRARIES})
   else()
      # foxy, galatic
      target_link_libraries(find_object ${LIBRARIES})
      rosidl_target_interfaces(find_object ${PROJECT_NAME} "rosidl_typesupport_cpp")

      function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
         rosidl_target_interfaces(${var} ${generate_interfaces_target} ${typesupport_name})
      endfunction()
      add_definitions(-DPRE_ROS_HUMBLE)
   endif()
ENDIF(COLCON_BUILD)

IF(NOT ROS_BUILD)
   INSTALL(TARGETS find_object
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
      
   install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
ELSEIF(CATKIN_BUILD)
   add_executable(find_object_2d ros1/find_object_2d_node.cpp)
   target_link_libraries(find_object_2d find_object ${LIBRARIES})

   add_executable(print_objects_detected ros1/print_objects_detected_node.cpp)
   target_link_libraries(print_objects_detected ${LIBRARIES})
   add_dependencies(print_objects_detected ${${PROJECT_NAME}_EXPORTED_TARGETS})

   add_executable(tf_example ros1/tf_example_node.cpp)
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
ELSE() # COLCON_BUILD
   add_executable(find_object_2d_node ros2/find_object_2d_node.cpp)
   set_target_properties(find_object_2d_node PROPERTIES  OUTPUT_NAME find_object_2d)
   ament_target_dependencies(find_object_2d_node ${AMENT_LIBRARIES})
   target_link_libraries(find_object_2d_node find_object ${LIBRARIES})
   
   add_executable(print_objects_detected ros2/print_objects_detected_node.cpp)
   ament_target_dependencies(print_objects_detected ${AMENT_LIBRARIES})
   target_link_libraries(print_objects_detected find_object ${LIBRARIES})
   
   add_executable(tf_example ros2/tf_example_node.cpp)
   ament_target_dependencies(tf_example ${AMENT_LIBRARIES})
   target_link_libraries(tf_example find_object ${LIBRARIES})
   
   # Only required when using messages built from the same package
   # https://index.ros.org/doc/ros2/Tutorials/Rosidl-Tutorial/
   get_default_rmw_implementation(rmw_implementation)
   find_package("${rmw_implementation}" REQUIRED)
   get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

   foreach(typesupport_impl ${typesupport_impls})
      rosidl_get_typesupport_target(find_object_2d_node
        ${PROJECT_NAME} ${typesupport_impl}
      )
      rosidl_get_typesupport_target(print_objects_detected
        ${PROJECT_NAME} ${typesupport_impl}
      )
      rosidl_get_typesupport_target(tf_example
        ${PROJECT_NAME} ${typesupport_impl}
      )
   endforeach()
   
   ## Mark executables and/or libraries for installation
   install(TARGETS 
      find_object
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
   )
   install(TARGETS 
      find_object_2d_node
      print_objects_detected
      tf_example
      DESTINATION lib/${PROJECT_NAME}
   )
ENDIF()

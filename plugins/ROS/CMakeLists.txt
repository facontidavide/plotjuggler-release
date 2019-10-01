
include_directories( ../  ${catkin_INCLUDE_DIRS}  )

# Qt related stuff
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)



#############
add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)
#############

SET( COMMON_SRC
    rule_editing.h
    rule_editing.cpp
    qnodedialog.h
    qnodedialog.cpp
    dialog_select_ros_topics.h
    dialog_select_ros_topics.cpp
    dialog_with_itemlist.h
    ./RosMsgParsers/ros_parser.h
    ./RosMsgParsers/ros_parser.cpp
    )

QT5_WRAP_UI ( COMMON_UI_SRC
    qnodedialog.ui
    rule_editing.ui
    dialog_select_ros_topics.ui
    dialog_with_itemlist.ui
    )

add_library( commonROS STATIC ${COMMON_SRC} ${COMMON_UI_SRC} )
target_link_libraries( commonROS
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Xml_LIBRARIES}
  ${catkin_LIBRARIES}
  fmt::fmt
  )


#############
SET( DATALOAD_SRC
    DataLoadROS/dataload_ros.cpp
    ../../include/PlotJuggler/dataloader_base.h
    )

add_library( DataLoadROS SHARED ${DATALOAD_SRC}  )
target_link_libraries( DataLoadROS  commonROS)

add_dependencies(DataLoadROS
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

#############
SET( DATASTREAM_SRC
    DataStreamROS/datastream_ROS.cpp
    DataStreamROS/datastream_ROS.h
    ../../include/PlotJuggler/datastreamer_base.h
    )

add_library( DataStreamROS SHARED ${DATASTREAM_SRC} )
target_link_libraries( DataStreamROS  commonROS )

add_dependencies(DataStreamROS
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

#############
SET( PUBLISHER_SRC
    TopicPublisherROS/statepublisher_rostopic.cpp
    TopicPublisherROS/statepublisher_rostopic.h
    ../../include/PlotJuggler/statepublisher_base.h
    )

QT5_WRAP_UI ( PUBLISHER_UI_SRC  )

add_library( RosTopicPublisher SHARED ${PUBLISHER_SRC} ${PUBLISHER_UI_SRC} )
target_link_libraries( RosTopicPublisher  ${catkin_LIBRARIES} commonROS )

#add_dependencies(RosTopicPublisher
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS}
#    )

#############
SET( ROSOUT_SRC
    RosoutPublisher/rosout_publisher.cpp
    RosoutPublisher/rosout_publisher.h
    RosoutPublisher/logs_table_model.cpp
    RosoutPublisher/logwidget.cpp
    RosoutPublisher/modelfilter.cpp
    ../../include/PlotJuggler/statepublisher_base.h
    )

QT5_WRAP_UI ( ROSOUT_UI RosoutPublisher/logwidget.ui )

add_library( RosoutPublisher SHARED ${ROSOUT_SRC}  ${ROSOUT_UI})
target_link_libraries( RosoutPublisher  ${catkin_LIBRARIES} commonROS )

add_dependencies(RosoutPublisher
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

if(COMPILING_WITH_CATKIN)
    install(TARGETS
        DataLoadROS
        DataStreamROS
        RosTopicPublisher
        RosoutPublisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
elseif()
    install(TARGETS DataLoadROS  DataStreamROS RosTopicPublisher RosoutPublisher
        DESTINATION bin  )
endif()

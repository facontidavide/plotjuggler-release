
include_directories( ../ ${catkin_INCLUDE_DIRS}  )

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
    ruleloaderwidget.h
    ruleloaderwidget.cpp
    qnodedialog.h
    qnodedialog.cpp
    )

QT5_WRAP_UI ( COMMON_UI_SRC
    qnodedialog.ui
    ruleloaderwidget.ui
    )

add_library( commonROS STATIC ${COMMON_SRC} ${COMMON_UI_SRC} )
target_link_libraries( commonROS
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Xml_LIBRARIES}
  ${catkin_LIBRARIES}
  )



#############
SET( DATALOAD_SRC
    DataLoadROS/dataload_ros.cpp
    DataLoadROS/dialog_select_ros_topics.h
    DataLoadROS/dialog_select_ros_topics.cpp
    )

QT5_WRAP_UI ( DATALOAD_UI_SRC
    DataLoadROS/dialog_select_ros_topics.ui
    ruleloaderwidget.ui
    )

add_library( DataLoadROS SHARED ${DATALOAD_SRC} ${DATALOAD_UI_SRC} )
target_link_libraries( DataLoadROS  commonROS)

add_dependencies(DataLoadROS
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

#############
SET( DATASTREAM_SRC
    DataStreamROS/datastream_ROS.cpp
    DataStreamROS/datastream_ROS.h
    DataStreamROS/rostopicselector.h
    DataStreamROS/rostopicselector.cpp
    )

QT5_WRAP_UI ( DATASTREAM_UI_SRC
    DataStreamROS/rostopicselector.ui )

add_library( DataStreamROS SHARED ${DATASTREAM_SRC} ${DATASTREAM_UI_SRC} )
target_link_libraries( DataStreamROS  commonROS )

add_dependencies(DataStreamROS
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

#############
SET( PUBLISHER_SRC
    TopicPublisherROS/statepublisher_rostopic.cpp
    TopicPublisherROS/statepublisher_rostopic.h
    )

QT5_WRAP_UI ( PUBLISHER_UI_SRC  )

add_library( RosTopicPublisher SHARED ${PUBLISHER_SRC} ${PUBLISHER_UI_SRC} )
target_link_libraries( RosTopicPublisher  ${catkin_LIBRARIES} commonROS )

add_dependencies(RosTopicPublisher
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation

if(catkin_FOUND)
    install(TARGETS DataLoadROS DataStreamROS RosTopicPublisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
elseif()
    install(TARGETS DataLoadROS DataStreamROS RosTopicPublisher DESTINATION bin  )
endif()

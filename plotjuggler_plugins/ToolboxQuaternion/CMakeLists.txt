
include_directories( ../ )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC
    quaternion_to_rpy.ui
    )

add_library(ToolboxQuaternion SHARED
    toolbox_quaternion.cpp
    quaternion_to_rpy.cpp
    ${UI_SRC}  )

target_link_libraries(ToolboxQuaternion
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    plotjuggler_base
    plotjuggler_qwt)

install(TARGETS ToolboxQuaternion DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}  )

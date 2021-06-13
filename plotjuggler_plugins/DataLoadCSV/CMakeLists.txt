
include_directories(  ../ )

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)

QT5_WRAP_UI ( UI_SRC  ../selectlistdialog.ui  )

SET( SRC
    dataload_csv.cpp
    ../selectlistdialog.h
    )

add_library(DataLoadCSV SHARED ${SRC} ${UI_SRC}  )
target_link_libraries(DataLoadCSV
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    plotjuggler_plugin_base)


install(TARGETS DataLoadCSV DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}  )


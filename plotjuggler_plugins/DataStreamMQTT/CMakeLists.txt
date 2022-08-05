
if(BUILDING_WITH_VCPKG)
    message(STATUS "Finding MOSQUITTO with vcpkg")
    find_package(mosquitto CONFIG REQUIRED)
    set(MOSQUITTO_FOUND ON)
elseif(BUILDING_WITH_CONAN)
    message(STATUS "Finding MOSQUITTO with conan")
    find_package(mosquitto CONFIG REQUIRED)
    set(MOSQUITTO_FOUND ON)
else()
    message(STATUS "Finding MOSQUITTO without package managers")
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
    find_package(Mosquitto REQUIRED)
endif()

if(MOSQUITTO_FOUND)

    message(STATUS "[MOSQUITTO] found.")
    add_definitions(${QT_DEFINITIONS})
    add_definitions(-DQT_PLUGIN)

    QT5_WRAP_UI ( UI_SRC  datastream_mqtt.ui  )

    SET( SRC
        datastream_mqtt.h
        datastream_mqtt.cpp
        mqtt_dialog.h
        mqtt_dialog.cpp
        mqtt_client.h
        mqtt_client.cpp
        )

    add_library(DataStreamMQTT_Mosquitto SHARED ${SRC} ${UI_SRC}  )

    target_link_libraries(DataStreamMQTT_Mosquitto
        ${Qt5Widgets_LIBRARIES}
        Qt5::Network
        Qt5::Svg
        ${MOSQUITTO_LIBRARIES}
        plotjuggler_base
        )
    target_include_directories(DataStreamMQTT_Mosquitto
        PUBLIC ${MOSQUITTO_INCLUDE_DIR})

    install(TARGETS DataStreamMQTT_Mosquitto
        DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}  )

else()
    message("[MOSQUITTO] not found. Skipping plugin DataStreamMQTT.")
endif()



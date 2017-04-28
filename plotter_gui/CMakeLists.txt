
include_directories( ./ ../  ../include  ../common)

QT5_ADD_RESOURCES (RES_SRC  resource.qrc )

QT5_WRAP_UI ( UI_SRC
    aboutdialog.ui
    mainwindow.ui
    removecurvedialog.ui
    busydialog.ui
    curvecolorpick.ui
    filterablelistwidget.ui
    tabbedplotwidget.ui
    axis_limits_dialog.ui
    ../common/selectlistdialog.ui
    )

SET( PLOTTER_SRC
    aboutdialog.cpp
    axis_limits_dialog.cpp
    main.cpp
    mainwindow.cpp
    plotwidget.cpp
    plotmatrix.cpp
    removecurvedialog.cpp
    plotmagnifier.cpp
    plotzoomer.cpp
    busydialog.cpp
    busytaskdialog.cpp
    customtracker.cpp
    curvecolorpick.cpp
    timeseries_qwt.cpp
    filterablelistwidget.cpp
    tabbedplotwidget.cpp
    subwindow.cpp

    mainwindow.h
    realslider.h
    utils.h
    ../common/selectlistdialog.h
    ../include/PlotJuggler/plotdata.h
    )

add_executable(PlotJuggler ${PLOTTER_SRC} ${RES_SRC} ${UI_SRC} ${BACKWARD_SRC})

if( Qt5Svg_FOUND)
    target_link_libraries(PlotJuggler
        Qt5::Core
        Qt5::Widgets
        Qt5::PrintSupport
        Qt5::Xml
        Qt5::Concurrent
        Qt5::Svg
        colorwidgets
        qwt_static
        ${BACKWARD_LIBS} )
else()
    target_link_libraries(PlotJuggler
        Qt5::Core
        Qt5::Widgets
        Qt5::PrintSupport
        Qt5::Xml
        Qt5::Concurrent
        colorwidgets
        qwt_static
        ${BACKWARD_LIBS})
endif()

if(COMPILING_WITH_CATKIN)
    install(TARGETS PlotJuggler
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
else()
    install(TARGETS PlotJuggler DESTINATION bin  )
endif()

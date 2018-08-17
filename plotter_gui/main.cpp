#include "mainwindow.h"
#include <QApplication>
#include <QSplashScreen>
#include <QThread>
#include <QCommandLineParser>
#include <QDesktopWidget>

QString getFunnySubtitle(){

    qsrand(time(NULL));
    int n = qrand() % 15;
    switch(n)
    {
    case 0: return "Now with less bugs than usual";
    case 1: return "Talk is cheap, show me the data!";
    case 2: return "The visualization tool that you deserve";
    case 3: return "Timeseries, timeseries everywhere!";
    case 4: return "Changing the world, one plot at a time";
    case 5: return "\"Harry Plotter\" was also an option";
    case 6: return "Add data and mix vigorously";
    case 7: return "Splashscreens make any app look better";
    case 8: return "I didn't find a better name...";
    case 9: return "\"It won't take long to code that\"..\n"
                "Davide, 2014";
    case 10: return "Visualize data responsibly";
    case 11: return "I don't always visualize data,\n"
                    "but when I do, I use PlotJuggler";
    }
    return "Juggle with data";
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("PlotJuggler");

    app.setOrganizationName("IcarusTechnology");
    app.setApplicationName("PlotJuggler");

    qApp->setStyleSheet(QString("QToolTip {\n"
                                "   border: 1px solid black;\n"
                                "   border-radius: 4px;\n"
                                "   background: white;\n"
                                "   color: black; }" ));

    QString VERSION_STRING = QString("%1.%2.%3").
            arg(PJ_MAJOR_VERSION).
            arg(PJ_MINOR_VERSION).
            arg(PJ_PATCH_VERSION);

    app.setApplicationVersion(VERSION_STRING);

    QCommandLineParser parser;
    parser.setApplicationDescription("PlotJuggler: the time series visualization tool that you deserve ");
    parser.addVersionOption();
    parser.addHelpOption();

    QCommandLineOption nosplash_option(QStringList() << "n" << "nosplash",
                                       QCoreApplication::translate("main", "Don't display the splashscreen"));
    parser.addOption(nosplash_option);

    QCommandLineOption test_option(QStringList() << "t" << "test",
                                   QCoreApplication::translate("main", "Generate test curves at startup"));
    parser.addOption(test_option);

    QCommandLineOption loadfile_option(QStringList() << "d" << "datafile",
                                       QCoreApplication::translate("main", "Load a file containing data"),
                                       QCoreApplication::translate("main", "file") );
    parser.addOption(loadfile_option);

    QCommandLineOption config_option(QStringList() << "l" << "layout",
                                     QCoreApplication::translate("main", "Load a file containing the layout configuration"),
                                     QCoreApplication::translate("main", "file") );
    parser.addOption(config_option);

    QCommandLineOption buffersize_option(QStringList() << "buffer_size",
                                     QCoreApplication::translate("main", "Change the maximum size of the streaming buffer (minimum: 10 default: 60)"),
                                     QCoreApplication::translate("main", "seconds") );
    parser.addOption(buffersize_option);

    parser.process( *qApp );

    /*
     * You, fearless code reviewer, decided to start a journey into my source code.
     * For your bravery, you deserve to know the truth, no matter how hard it is to accept it.
     * The splashscreen is useless; not only it is useless, it will make your start-up
     * time slower by a couple of seconds for absolutely no reason.
     * But what are two seconds compared with the time that PlotJuggler will save you?
     * The splashscreen is the connection between me and my users, the glue that keeps
     * together our invisible relationship.
     * Now it is up to you to decide: you can block the splashscreen forever or not,
     * reject a message that brings a little of happiness into your day, spent analyzing data.
     * Please don't do it.
     */

    if( parser.isSet(nosplash_option) == false)
    // if(false) // if you uncomment this line, a kitten will die somewhere in the world.
    {
        QPixmap main_pixmap(":/splash/resources/splash.jpg");

        QPainter painter;
        painter.begin( &main_pixmap);
        painter.setPen(QColor(77, 77, 77));

        QString subtitle = getFunnySubtitle();
        int font_size = 34;
        do{
            painter.setFont( QFont("Arial", font_size-- ) );
        }while(font_size > 22 && painter.fontMetrics().width(subtitle) > 600 );

        painter.drawText( QRect(20, 130, 620, 200),
                          Qt::AlignHCenter | Qt::AlignVCenter, subtitle );
        painter.end();

        QSplashScreen splash(main_pixmap);
        QDesktopWidget* desktop = QApplication::desktop();
        const int scrn = desktop->screenNumber(QCursor::pos());
        const QPoint currentDesktopsCenter = desktop->availableGeometry(scrn).center();
        splash.move(currentDesktopsCenter - splash.rect().center());

        splash.show();
        app.processEvents();
        splash.raise();

        const auto deadline = QDateTime::currentDateTime().addMSecs( 100*(20 + subtitle.size()*0.4) );

        MainWindow w( parser );

        while( QDateTime::currentDateTime() < deadline && !splash.isHidden() )
        {
            app.processEvents();
            QThread::msleep(100);
            splash.raise();
        }

        splash.close();
        w.show();
        return app.exec();
    }
    else{
        MainWindow w( parser );
        w.show();
        return app.exec();
    }

    return -1;
}

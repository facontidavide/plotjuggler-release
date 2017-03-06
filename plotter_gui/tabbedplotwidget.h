#ifndef TABBEDPLOTWIDGET_H
#define TABBEDPLOTWIDGET_H

#include <QWidget>
#include <QMainWindow>
#include <QTableWidget>
#include "plotmatrix.h"

namespace Ui {
class TabbedPlotWidget;
}


class TabbedPlotWidget : public QWidget
{
    Q_OBJECT

public:
    typedef struct{} MainWindowArea;

    explicit TabbedPlotWidget(QMainWindow *main_window,
                              PlotDataMap *mapped_data,
                              QMainWindow *parent );

    void setSiblingsList( const std::map<QString,TabbedPlotWidget*>& other_tabbed_widgets );

    PlotMatrix* currentTab();

    QTabWidget* tabWidget();

    void addTab(PlotMatrix *tab = NULL);

    QDomElement xmlSaveState(QDomDocument &doc) const;
    bool xmlLoadState(QDomElement &tabbed_area);

    ~TabbedPlotWidget();

public slots:

    void setStreamingMode(bool streaming_mode);

private slots:

    void on_renameCurrentTab();

    void on_savePlotsToFile();

    void on_pushAddColumn_pressed();

    void on_pushVerticalResize_pressed();

    void on_pushHorizontalResize_pressed();

    void on_pushAddRow_pressed();

    void on_addTabButton_pressed();

    void on_pushRemoveEmpty_pressed();

    void on_tabWidget_currentChanged(int index);

    void on_tabWidget_tabCloseRequested(int index);

    void on_buttonLinkHorizontalScale_toggled(bool checked);

    void on_requestTabMovement(const QString &destination_name);

    void on_moveTabIntoNewWindow();

    void on_pushButtonShowLabel_toggled(bool checked);

    void on_pushButtonShowGrid_toggled(bool checked);

private:
    Ui::TabbedPlotWidget *ui;

    QAction* _action_renameTab;
    QAction* _action_savePlots;

    QMenu* _tab_menu;

    PlotDataMap *_mapped_data;

    bool _horizontal_link;

    QString _parent_type;

    std::map<QString,TabbedPlotWidget*> _other_siblings;

protected:
    virtual bool eventFilter(QObject *obj, QEvent *event);

signals:
    void created();
    void undoableChangeHappened();
    void matrixAdded( PlotMatrix * );
    void sendTabToNewWindow(PlotMatrix *);
};

#endif // TABBEDPLOTWIDGET_H

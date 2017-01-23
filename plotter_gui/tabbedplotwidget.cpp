#include <QMenu>
#include <QSignalMapper>
#include <QAction>
#include <QInputDialog>
#include <QMouseEvent>
#include "tabbedplotwidget.h"
#include "ui_tabbedplotwidget.h"


TabbedPlotWidget::TabbedPlotWidget(QMainWindow *main_window, PlotDataMap *mapped_data, QMainWindow *parent ) :
    QWidget(parent),
    ui(new Ui::TabbedPlotWidget)
{
    _mapped_data = mapped_data;

    if( main_window == parent){
       _parent_type = QString("main_window");
    }
    else{
      _parent_type = QString("floating_window");
    }
    ui->setupUi(this);

    _horizontal_link = true;

    ui->tabWidget->tabBar()->installEventFilter( this );

    _action_renameTab = new QAction(tr("Rename tab"), this);

    connect( _action_renameTab, SIGNAL(triggered()), this, SLOT(on_renameCurrentTab()) );

    _tab_menu = new QMenu(this);
    _tab_menu->addAction( _action_renameTab );
    _tab_menu->addSeparator();

    connect( this, SIGNAL(destroyed(QObject*)),             main_window, SLOT(on_tabbedAreaDestroyed(QObject*)) );
    connect( this, SIGNAL(sendTabToNewWindow(PlotMatrix*)), main_window, SLOT(onCreateFloatingWindow(PlotMatrix*)) );
    connect( this, SIGNAL(matrixAdded(PlotMatrix*)),        main_window, SLOT(onPlotMatrixAdded(PlotMatrix*)) );
    connect( this, SIGNAL(undoableChangeHappened()),        main_window, SLOT(onUndoableChange()) );

    this->addTab();
}

void TabbedPlotWidget::setSiblingsList(const std::map<QString, TabbedPlotWidget *> &other_tabbed_widgets)
{
    _other_siblings = other_tabbed_widgets;
}

PlotMatrix *TabbedPlotWidget::currentTab()
{
    return static_cast<PlotMatrix*>( ui->tabWidget->currentWidget() );
}

QTabWidget *TabbedPlotWidget::tabWidget()
{
    return ui->tabWidget;
}

void TabbedPlotWidget::addTab( PlotMatrix* tab)
{
    if( !tab )
    {
        tab = new PlotMatrix("plot", _mapped_data, this);
        ui->tabWidget->addTab( tab, QString("plot") );

        QApplication::processEvents();
        emit matrixAdded( tab );
        tab->addColumn();
    }
    else{
        ui->tabWidget->addTab( tab, tab->name() );
    }

    ui->tabWidget->setCurrentWidget( tab );

    tab->setHorizontalLink( _horizontal_link );
}

QDomElement TabbedPlotWidget::xmlSaveState(QDomDocument &doc) const
{
    QDomElement tabbed_area = doc.createElement( "tabbed_widget" );

    tabbed_area.setAttribute("parent", _parent_type);

    for(int i=0; i< ui->tabWidget->count(); i++)
    {
        PlotMatrix* widget = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );
        QDomElement element = widget->xmlSaveState(doc);

        element.setAttribute("tab_name",  ui->tabWidget->tabText(i) );
        tabbed_area.appendChild( element );
    }

    QDomElement current_plotmatrix =  doc.createElement( "currentPlotMatrix" );
    current_plotmatrix.setAttribute( "index", ui->tabWidget->currentIndex() );
    tabbed_area.appendChild( current_plotmatrix );

    return tabbed_area;
}

bool TabbedPlotWidget::xmlLoadState(QDomElement &tabbed_area)
{
    int num_tabs =  ui->tabWidget->count();
    int index = 0;

    QDomElement plotmatrix_el;

    for (  plotmatrix_el = tabbed_area.firstChildElement( "plotmatrix" )  ;
           !plotmatrix_el.isNull();
           plotmatrix_el = plotmatrix_el.nextSiblingElement( "plotmatrix" ) )
    {
        // add if tabs are too few
        if( index == num_tabs)
        {
            this->addTab( NULL );
            num_tabs++;
        }
        PlotMatrix* plot_matrix = static_cast<PlotMatrix*>(  ui->tabWidget->widget(index) );
        bool success = plot_matrix->xmlLoadState( plotmatrix_el );

        // read tab name
        if( plotmatrix_el.hasAttribute("tab_name"))
        {
            QString tab_name = plotmatrix_el.attribute("tab_name" );
            ui->tabWidget->setTabText( index, tab_name );
            plot_matrix->setName( tab_name );
        }

        if( !success )
        {
            return false;
        }

        index++;
    }

    // remove if tabs are too much
    while( num_tabs > index ){
        ui->tabWidget->removeTab( num_tabs-1 );
        num_tabs--;
    }

    QDomElement current_plotmatrix =  tabbed_area.firstChildElement( "currentPlotMatrix" );
    int current_index = current_plotmatrix.attribute( "index" ).toInt();
    ui->tabWidget->setCurrentIndex( current_index );

    currentTab()->replot();
    return true;
}

void TabbedPlotWidget::setStreamingMode(bool streaming_mode)
{
    ui->buttonLinkHorizontalScale->setEnabled( !streaming_mode );
    ui->pushVerticalResize->setEnabled( !streaming_mode );
    ui->pushHorizontalResize->setEnabled( !streaming_mode );
}


TabbedPlotWidget::~TabbedPlotWidget(){
    delete ui;
}

void TabbedPlotWidget::on_renameCurrentTab()
{
    int idx = ui->tabWidget->tabBar()->currentIndex ();

    bool ok = true;
    QString newName = QInputDialog::getText (
                this, tr ("Change Name of the selected tab"),
                tr ("Insert New Tab Name"),
                QLineEdit::Normal,
                ui->tabWidget->tabText (idx),
                &ok);

    if (ok) {
        ui->tabWidget->setTabText (idx, newName);
        currentTab()->setName( newName );
    }
}

void TabbedPlotWidget::on_pushAddColumn_pressed()
{
    currentTab()->addColumn();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushVerticalResize_pressed()
{
    currentTab()->maximumZoomOutVertical();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushHorizontalResize_pressed()
{
    currentTab()->maximumZoomOutHorizontal();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushAddRow_pressed()
{
    currentTab()->addRow();
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_addTabButton_pressed()
{
    addTab( NULL );
    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_pushRemoveEmpty_pressed()
{
    PlotMatrix *tab = currentTab();

    for( unsigned row = 0; row< tab->rowsCount(); row++)
    {
        while( tab->rowsCount() > 1 &&
               tab->isRowEmpty( row ) &&
               row < tab->rowsCount() )
        {
            tab->removeRow( row );
        }
    }

    for( unsigned col = 0; col< tab->colsCount(); col++)
    {
        while( tab->colsCount() > 1 &&
               tab->isColumnEmpty( col ) &&
               col < tab->colsCount() )
        {
            tab->removeColumn( col );
        }
    }

    emit undoableChangeHappened();
}

void TabbedPlotWidget::on_tabWidget_currentChanged(int index)
{
    if( ui->tabWidget->count() == 0)
    {
        if( _parent_type.compare("main_window") == 0)
        {
            addTab( NULL);
        }
        else{
            this->parent()->deleteLater();
        }
    }

    PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );
    if( tab )
    {
        tab->replot();
    }
}

void TabbedPlotWidget::on_tabWidget_tabCloseRequested(int index)
{
    PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(index) );

    bool ask_confirmation = true;
    if( tab->plotCount() == 1 )
    {
        if( tab->plotAt(0)->isEmpty()){
            ask_confirmation = false;
        }
    }

    QMessageBox::StandardButton do_remove = QMessageBox::Yes;

    if( ask_confirmation )
    {
        ui->tabWidget->setCurrentIndex( index );
        QApplication::processEvents();

        do_remove = QMessageBox::question(0, tr("Warning"),
                                          tr("Do you really want to destroy this tab?\n"),
                                          QMessageBox::Yes | QMessageBox::No,
                                          QMessageBox::No );
    }
    if( do_remove == QMessageBox::Yes )
    {
        // first add then delete.
        // Otherwise currentPlotGrid might be empty
        if( ui->tabWidget->count() == 1){
            on_addTabButton_pressed();
        }
        ui->tabWidget->removeTab( index );
        emit undoableChangeHappened();
    }
}

void TabbedPlotWidget::on_buttonLinkHorizontalScale_toggled(bool checked)
{
    _horizontal_link = checked;

    for (int i = 0; i < ui->tabWidget->count(); i++)
    {
        PlotMatrix* tab = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );
        tab->setHorizontalLink( _horizontal_link );
    }
}

void TabbedPlotWidget::on_requestTabMovement(const QString & destination_name)
{
    TabbedPlotWidget* destination_widget = _other_siblings[destination_name];

    PlotMatrix* tab_to_move = currentTab();
    int index = ui->tabWidget->tabBar()->currentIndex ();

    const QString& tab_name =  this->tabWidget()->tabText(index);

    destination_widget->tabWidget()->addTab( tab_to_move, tab_name );

    // tab_to_move->setParent( destination_widget );

    qDebug() << "move "<< tab_name<< " into " << destination_name;
    emit undoableChangeHappened();

}

void TabbedPlotWidget::on_moveTabIntoNewWindow()
{
    emit sendTabToNewWindow( currentTab() );
}


bool TabbedPlotWidget::eventFilter(QObject *obj, QEvent *event)
{
    QTabBar* tab_bar = ui->tabWidget->tabBar();

    if (obj == tab_bar )
    {
        if( event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent *mouse_event = static_cast<QMouseEvent*>(event);

            int index = tab_bar->tabAt( mouse_event->pos() );
            tab_bar->setCurrentIndex( index );


            if( mouse_event->button() == Qt::RightButton )
            {
                QMenu* submenu = new QMenu("Move tab to...");
                _tab_menu->addMenu( submenu );

                std::map<QString,TabbedPlotWidget*>::iterator it;
                QSignalMapper* signalMapper = new QSignalMapper(submenu);

                //-----------------------------------
                QAction* action_new_window = submenu->addAction( "New Window" );

                QIcon icon;
                icon.addFile(QStringLiteral(":/icons/resources/stacks_32px.png"), QSize(16, 16), QIcon::Normal, QIcon::Off);

                action_new_window->setIcon( icon);
                submenu->addSeparator();

                connect( action_new_window, SIGNAL(triggered()), this, SLOT(on_moveTabIntoNewWindow() ));

                //-----------------------------------
                for ( it = _other_siblings.begin(); it != _other_siblings.end(); it++)
                {
                    QString name = it->first;
                    TabbedPlotWidget* tabbed_menu = it->second;
                    if( tabbed_menu != this )
                    {
                        QAction* action = submenu->addAction( name );
                        connect(action, SIGNAL(triggered()), signalMapper, SLOT(map()));
                        signalMapper->setMapping( action, name );
                    }
                }

                connect(signalMapper, SIGNAL(mapped(const QString &)),
                        this, SLOT(on_requestTabMovement(const QString &)));

                //-------------------------------
                _tab_menu->exec( mouse_event->globalPos() );
                //-------------------------------
                submenu->deleteLater();
            }
        }
    }

    // Standard event processing
    return QObject::eventFilter(obj, event);
}

void TabbedPlotWidget::on_pushButtonShowLabel_toggled(bool checked)
{    
    for(int i=0; i< ui->tabWidget->count(); i++)
    {
        PlotMatrix* matrix = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );

        for(unsigned p=0; p< matrix->plotCount(); p++)
        {
            PlotWidget* plot = matrix->plotAt(p);
            plot->activateLegent( checked );
        }
    }
    currentTab()->replot();
}



void TabbedPlotWidget::on_pushButtonShowGrid_toggled(bool checked)
{
    for(int i=0; i< ui->tabWidget->count(); i++)
    {
        PlotMatrix* matrix = static_cast<PlotMatrix*>( ui->tabWidget->widget(i) );

        for(unsigned p=0; p< matrix->plotCount(); p++)
        {
            PlotWidget* plot = matrix->plotAt(p);
            plot->activateGrid( checked );
        }
    }
    currentTab()->replot();
}

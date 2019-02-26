#include "plotwidget.h"
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <qwt_scale_widget.h>
#include <qwt_plot_canvas.h>
#include <qwt_scale_engine.h>
#include <qwt_plot_layout.h>
#include <qwt_scale_draw.h>
#include <QAction>
#include <QMessageBox>
#include <QMenu>
#include <QPushButton>
#include <iostream>
#include <limits>
#include "removecurvedialog.h"
#include "curvecolorpick.h"
#include <QApplication>
#include <set>
#include <memory>
#include <qwt_text.h>
#include <QActionGroup>
#include <QWheelEvent>
#include <QFileDialog>
#include <QSettings>
#include <QtXml/QDomElement>
#include "qwt_plot_renderer.h"
#include "qwt_series_data.h"
#include "PlotJuggler/random_color.h"
#include "point_series_xy.h"
#include "transforms/custom_function.h"
#include "transforms/custom_timeseries.h"

const double MAX_DOUBLE = std::numeric_limits<double>::max() / 2 ;

static const char* noTransform = "noTransform";
static const char* Derivative1st = "1st Derivative";
static const char* Derivative2nd = "2nd Derivative";
static bool if_xy_plot_failed_show_dialog = true;

static QStringList builtin_trans = {
    noTransform,
    Derivative1st,
    Derivative2nd
};

PlotWidget::PlotWidget(PlotDataMapRef &datamap, QWidget *parent):
    QwtPlot(parent),
    _zoomer( nullptr ),
    _magnifier( nullptr ),
    _panner1( nullptr ),
    _panner2( nullptr ),
    _tracker ( nullptr ),
    _legend( nullptr ),
    _grid( nullptr ),
    _mapped_data( datamap ),
    _dragging( { DragInfo::NONE, {}, nullptr } ),
    _show_line_and_points(false),
    _time_offset(0.0),
    _axisX(nullptr),
    _transform_select_dialog(nullptr),
    _zoom_enabled(true)
{
    this->setAcceptDrops( true );

    this->setMinimumWidth( 100 );
    this->setMinimumHeight( 100 );

    this->sizePolicy().setHorizontalPolicy( QSizePolicy::Expanding);
    this->sizePolicy().setVerticalPolicy( QSizePolicy::Expanding);

    QwtPlotCanvas *canvas = new QwtPlotCanvas(this);

    canvas->setFrameStyle( QFrame::NoFrame );
    canvas->setPaintAttribute( QwtPlotCanvas::BackingStore, true );

    this->setCanvas( canvas );
    this->setCanvasBackground( Qt::white );

    this->setAxisAutoScale(QwtPlot::yLeft, true);
    this->setAxisAutoScale(QwtPlot::xBottom, true);

    this->axisScaleEngine(QwtPlot::xBottom)->setAttribute(QwtScaleEngine::Floating,true);
    this->plotLayout()->setAlignCanvasToScales( true );

    //--------------------------
    _grid = new QwtPlotGrid();
    _zoomer = ( new PlotZoomer( this->canvas() ) );
    _magnifier = ( new PlotMagnifier( this->canvas() ) );
    _panner1 = ( new QwtPlotPanner( this->canvas() ) );
    _panner2 = ( new QwtPlotPanner( this->canvas() ) );
    _tracker = ( new CurveTracker( this ) );

    _grid->setPen(QPen(Qt::gray, 0.0, Qt::DotLine));

    _zoomer->setRubberBandPen( QColor( Qt::red , 1, Qt::DotLine) );
    _zoomer->setTrackerPen( QColor( Qt::green, 1, Qt::DotLine ) );
    _zoomer->setMousePattern( QwtEventPattern::MouseSelect1, Qt::LeftButton, Qt::NoModifier );
    connect(_zoomer,  &PlotZoomer::zoomed, this, &PlotWidget::on_externallyResized );

    _magnifier->setAxisEnabled(xTop, false);
    _magnifier->setAxisEnabled(yRight, false);

    // disable right button. keep mouse wheel
    _magnifier->setMouseButton( Qt::NoButton );
    connect(_magnifier, &PlotMagnifier::rescaled, this, &PlotWidget::on_externallyResized );
    connect(_magnifier, &PlotMagnifier::rescaled, this, &PlotWidget::replot );

    _panner1->setMouseButton( Qt::LeftButton, Qt::ControlModifier);
    _panner2->setMouseButton( Qt::MiddleButton, Qt::NoModifier);

    connect(_panner1, &QwtPlotPanner::panned, this, &PlotWidget::on_panned );
    connect(_panner2, &QwtPlotPanner::panned, this, &PlotWidget::on_panned );

    //-------------------------

    buildActions();
    buildLegend();

    this->canvas()->setMouseTracking(true);

    setDefaultRangeX();

    _axis_limits_dialog = new AxisLimitsDialog(this);

    _custom_Y_limits.min = (-MAX_DOUBLE );
    _custom_Y_limits.max = ( MAX_DOUBLE );
}

void PlotWidget::buildActions()
{

    QIcon iconDeleteList;

    auto getActionAndIcon = [this](const char* text, const char* file)
    {
        QIcon icon;
        icon.addFile( tr(file), QSize(30,30));
        auto action = new QAction( tr(text), this);
        action->setIcon(icon);
        return action;
    };

    _action_removeCurve = getActionAndIcon("&Remove curves",
                                           ":/icons/resources/light/remove_list.png" );
    _action_removeCurve->setStatusTip(tr("Remove one or more curves from this plot"));
    connect(_action_removeCurve, &QAction::triggered, this, &PlotWidget::launchRemoveCurveDialog);

    _action_removeAllCurves = getActionAndIcon("&Remove ALL curves",
                                               ":/icons/resources/light/remove.png" );
    connect(_action_removeAllCurves, &QAction::triggered, this, &PlotWidget::detachAllCurves);
    connect(_action_removeAllCurves, &QAction::triggered, this, &PlotWidget::undoableChange );

    _action_changeColorsDialog = getActionAndIcon("&Change colors",
                                                  ":/icons/resources/light/colored_charts.png" );
    _action_changeColorsDialog->setStatusTip(tr("Change the color of the curves"));
    connect(_action_changeColorsDialog, &QAction::triggered, this, &PlotWidget::on_changeColorsDialog_triggered);

    _action_showPoints = getActionAndIcon("&Show lines and points",
                                          ":/icons/resources/light/point_chart.png" );
    _action_showPoints->setCheckable( true );
    _action_showPoints->setChecked( false );
    connect(_action_showPoints, &QAction::triggered, this, &PlotWidget::on_showPoints_triggered);

    _action_editLimits = new  QAction(tr("&Edit Axis Limits"), this);
    connect(_action_editLimits, &QAction::triggered, this, &PlotWidget::on_editAxisLimits_triggered);

    _action_zoomOutMaximum = getActionAndIcon("&Zoom Out", ":/icons/resources/light/zoom_max.png" );
    connect(_action_zoomOutMaximum, &QAction::triggered, this, [this]()
            {
                zoomOut(true);
                replot();
                emit undoableChange();
            });

    _action_zoomOutHorizontally = getActionAndIcon("&Zoom Out Horizontally",
                                                   ":/icons/resources/light/zoom_horizontal.png" );
    connect(_action_zoomOutHorizontally, &QAction::triggered, this, [this]()
    {
        on_zoomOutHorizontal_triggered(true);
        replot();
        emit undoableChange();
    });

    _action_zoomOutVertically = getActionAndIcon("&Zoom Out Vertically",
                                                 ":/icons/resources/light/zoom_vertical.png" );
    connect(_action_zoomOutVertically, &QAction::triggered, this, [this]()
    {
        on_zoomOutVertical_triggered(true);
        replot();
        emit undoableChange();
    });

    QFont font;
    font.setPointSize(10);

    _action_noTransform = new QAction(tr("&NO Transform"), this);
    _action_noTransform->setCheckable( true );
    _action_noTransform->setChecked( true );
    connect(_action_noTransform, &QAction::triggered, this, [this, font]()
    {
        QwtText text("");
        text.setFont(font);
        this->setFooter(text);
        this->on_changeToBuiltinTransforms(noTransform);
    } );

    _action_1stDerivativeTransform = new QAction(tr("&1st Derivative"), this);
    _action_1stDerivativeTransform->setCheckable( true );
    connect(_action_1stDerivativeTransform, &QAction::triggered, this, [this, font]()
    {
        QwtText text("1st Derivative");
        text.setFont(font);
        this->setFooter(text);
        this->on_changeToBuiltinTransforms(Derivative1st);
    } );

    _action_2ndDerivativeTransform = new QAction(tr("&2nd Derivative"), this);
    _action_2ndDerivativeTransform->setCheckable( true );
    connect(_action_2ndDerivativeTransform, &QAction::triggered, this, [this, font]()
    {
        QwtText text("2nd Derivative");
        text.setFont(font);
        this->setFooter(text);
        this->on_changeToBuiltinTransforms(Derivative2nd);
    } );

    _action_phaseXY = new QAction(tr("&XY plot"), this);
    _action_phaseXY->setCheckable( true );
    _action_phaseXY->setEnabled(false);
    connect(_action_phaseXY, &QAction::triggered, this, &PlotWidget::on_convertToXY_triggered);

    _action_custom_transform = new QAction(tr("&Custom..."), this);
    _action_custom_transform->setCheckable( true );
    connect(_action_custom_transform, &QAction::triggered,
            this, &PlotWidget::on_customTransformsDialog);

    _action_saveToFile = getActionAndIcon("&Save plot to file",
                                          ":/icons/resources/light/save.png" );
    connect(_action_saveToFile, &QAction::triggered, this, &PlotWidget::on_savePlotToFile);

    auto transform_group = new QActionGroup(this);

    transform_group->addAction(_action_noTransform);
    transform_group->addAction(_action_1stDerivativeTransform);
    transform_group->addAction(_action_2ndDerivativeTransform);
    transform_group->addAction(_action_phaseXY);
    transform_group->addAction(_action_custom_transform);
}


void PlotWidget::canvasContextMenuTriggered(const QPoint &pos)
{
    QString edit("&Edit Axis Limits ");
    edit.append( _axis_limits_dialog->limitsEnabled() ? tr("(ENABLED)") : tr("(disabled)") ) ;
    _action_editLimits->setText( edit );

    QMenu menu(this);
    menu.addAction(_action_removeCurve);
    menu.addAction(_action_removeAllCurves);
    menu.addSeparator();
    menu.addAction(_action_changeColorsDialog);
    menu.addAction(_action_showPoints);
    menu.addSeparator();
    menu.addAction(_action_editLimits);
    menu.addAction(_action_zoomOutMaximum);
    menu.addAction(_action_zoomOutHorizontally);
    menu.addAction(_action_zoomOutVertically);
    menu.addSeparator();
    menu.addAction( _action_noTransform );
    menu.addAction( _action_1stDerivativeTransform );
    menu.addAction( _action_2ndDerivativeTransform );
    menu.addAction( _action_phaseXY );
    menu.addAction( _action_custom_transform );
    menu.addSeparator();
    menu.addAction( _action_saveToFile );

    _action_removeCurve->setEnabled( ! _curve_list.empty() );
    _action_removeAllCurves->setEnabled( ! _curve_list.empty() );
    _action_changeColorsDialog->setEnabled(  ! _curve_list.empty() );
    _action_phaseXY->setEnabled( _axisX != nullptr );

    if( !_axisX )
    {
        menu.setToolTipsVisible(true);
        _action_phaseXY->setToolTip(
                    "To show a XY plot, you must first provide the X axis.\n"
                    "Drag andn drop a curve using the RIGHT mouse\n"
                    "button instead of the left one." );
    }

    menu.exec( canvas()->mapToGlobal(pos) );
}


void PlotWidget::buildLegend()
{
    _legend = new QwtPlotLegendItem();
    _legend->attach( this );

    _legend->setRenderHint( QwtPlotItem::RenderAntialiased );
    QColor color( Qt::black );
    _legend->setTextPen( color );
    _legend->setBorderPen( color );
    QColor c( Qt::white );
    c.setAlpha( 200 );
    _legend->setBackgroundBrush( c );

    _legend->setMaxColumns( 1 );
    _legend->setAlignment( Qt::Alignment( Qt::AlignTop | Qt::AlignRight ) );
    _legend->setBackgroundMode( QwtPlotLegendItem::BackgroundMode::LegendBackground   );

    _legend->setBorderRadius( 6 );
    _legend->setMargin( 1 );
    _legend->setSpacing( 1 );
    _legend->setItemMargin( 1 );

    QFont font = _legend->font();
    font.setPointSize( 9 );
    _legend->setFont( font );
    _legend->setVisible( true );
}



PlotWidget::~PlotWidget()
{

}

bool PlotWidget::addCurve(const std::string &name)
{
    auto it = _mapped_data.numeric.find( name );
    if( it == _mapped_data.numeric.end())
    {
        return false;
    }

    if( _curve_list.find(name) != _curve_list.end())
    {
        return false;
    }

    PlotData& data = it->second;
    const auto qname = QString::fromStdString( name );

    auto curve = new QwtPlotCurve( qname );
    try {
        auto plot_qwt = createSeriesData( _default_transform, &data );
        _curves_transform.insert( {name, _default_transform} );

        curve->setPaintAttribute( QwtPlotCurve::ClipPolygons, true );
        curve->setPaintAttribute( QwtPlotCurve::FilterPointsAggressive, true );
        curve->setData( plot_qwt );

    }
    catch( std::exception& ex)
    {
        QMessageBox::warning(this, "Exception!", ex.what());
        return false;
    }

    if( _show_line_and_points ) {
        curve->setStyle( QwtPlotCurve::LinesAndDots);
    }
    else{
        curve->setStyle( QwtPlotCurve::Lines);
    }

    QColor color = data.getColorHint();
    if( color == Qt::black)
    {
        color = randomColorHint();
        data.setColorHint(color);
    }
    curve->setPen( color,  0.8 );
    curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );

    curve->attach( this );
    _curve_list.insert( std::make_pair(name, curve));

    auto marker = new QwtPlotMarker;
    _point_marker.insert( std::make_pair(name, marker) );
    marker->attach( this );
    marker->setVisible( isXYPlot() );

    QwtSymbol *sym = new QwtSymbol(
                QwtSymbol::Diamond,
                Qt::red, color,
                QSize(10,10));

    marker->setSymbol(sym);

    return true;
}

void PlotWidget::removeCurve(const std::string &curve_name)
{
    auto it = _curve_list.find(curve_name);
    if( it != _curve_list.end() )
    {
        auto& curve = it->second;
        curve->detach();
        _curve_list.erase( it );

        auto marker_it = _point_marker.find(curve_name);
        if( marker_it != _point_marker.end() )
        {
            auto marker = marker_it->second;
            if( marker ){
                marker->detach();
            }
            _point_marker.erase(marker_it);
        }

        emit curveListChanged();
    }
    _curves_transform.erase( curve_name );

    if( isXYPlot() && _axisX && _axisX->name() == curve_name)
    {
        // Without the X axis, transform all the curves to noTransform
        _axisX = nullptr;
        _default_transform.clear();
        for(auto& it : _curve_list)
        {
            auto& curve = it.second;

            auto data_it = _mapped_data.numeric.find( curve_name );
            if( data_it != _mapped_data.numeric.end())
            {
                const auto& data = data_it->second;
                auto data_series = createSeriesData( _default_transform, &data);
                curve->setData( data_series );
            }
        }
        on_changeToBuiltinTransforms( _default_transform );
        emit curveListChanged();
    }
}

bool PlotWidget::isEmpty() const
{
    return _curve_list.empty();
}

const std::map<std::string, QwtPlotCurve*> &PlotWidget::curveList() const
{
    return _curve_list;
}

void PlotWidget::dragEnterEvent(QDragEnterEvent *event)
{
    changeBackgroundColor( QColor( 230, 230, 230 ) );

    const QMimeData *mimeData = event->mimeData();
    QStringList mimeFormats = mimeData->formats();
    for(const QString& format: mimeFormats)
    {
        QByteArray encoded = mimeData->data( format );
        QDataStream stream(&encoded, QIODevice::ReadOnly);
        _dragging.curves.clear();
        _dragging.source = event->source();

        while (!stream.atEnd())
        {
            QString curve_name;
            stream >> curve_name;
            _dragging.curves.push_back( curve_name );
        }

        if( format.contains( "curveslist/add_curve") )
        {
            _dragging.mode = DragInfo::CURVES;
            event->acceptProposedAction();
        }
        if( format.contains( "curveslist/new_X_axis") && _dragging.curves.size() == 1 )
        {
            _dragging.mode = DragInfo::NEW_X;
            event->acceptProposedAction();
        }
        if( format.contains( "plot_area")  )
        {
            if(_dragging.curves.size() == 1 &&
                    windowTitle() != _dragging.curves.front() )
            {
                _dragging.mode = DragInfo::SWAP_PLOTS;
                event->acceptProposedAction();
            }
        }
    }
}


void PlotWidget::dragLeaveEvent(QDragLeaveEvent*)
{
    QPoint local_pos =  canvas()->mapFromGlobal(QCursor::pos()) ;
    // prevent spurious exits
    if( canvas()->rect().contains( local_pos ))
    {
        // changeBackgroundColor( QColor( 250, 150, 150 ) );
    }
    else{
        changeBackgroundColor( Qt::white );
        _dragging.mode = DragInfo::NONE;
        _dragging.curves.clear();
    }
}

void PlotWidget::dropEvent(QDropEvent *)
{
    bool curves_changed = false;
    bool background_changed = false;

    if( _dragging.mode == DragInfo::CURVES)
    {
        for( const auto& curve_name : _dragging.curves)
        {
            bool added = addCurve( curve_name.toStdString() );
            curves_changed = curves_changed || added;
        }
        emit curvesDropped();
    }
    else if( _dragging.mode == DragInfo::NEW_X)
    {
        changeAxisX( _dragging.curves.front() );
        curves_changed = true;
        emit curvesDropped();
    }
    else if( _dragging.mode == DragInfo::SWAP_PLOTS )
    {
        auto plot_widget = dynamic_cast<PlotWidget*>(_dragging.source);
        if( plot_widget )
        {
            emit swapWidgetsRequested( plot_widget, this );
        }
    }
    if( _dragging.mode != DragInfo::NONE &&
            canvasBackground().color() != Qt::white)
    {
        this->setCanvasBackground( Qt::white );
        background_changed = true;
    }

    if( curves_changed )
    {
        zoomOut(false);
        emit curveListChanged();
        emit undoableChange();
    }
    if( curves_changed || background_changed)
    {
        replot();
    }
    _dragging.mode = DragInfo::NONE;
    _dragging.curves.clear();
}

void PlotWidget::detachAllCurves()
{
    for(auto& it: _curve_list)   { it.second->detach(); }
    for(auto& it: _point_marker) { it.second->detach(); }

    if( isXYPlot() )
    {
        _axisX = nullptr;
        _action_noTransform->trigger();
    }

    _curve_list.clear();
    _curves_transform.clear();
    _point_marker.clear();
    emit _tracker->setPosition( _tracker->actualPosition() );

    emit curveListChanged();

    replot();
}

void PlotWidget::on_panned(int , int )
{
    on_externallyResized(currentBoundingRect());
}

QDomElement PlotWidget::xmlSaveState( QDomDocument &doc) const
{
    QDomElement plot_el = doc.createElement("plot");

    QDomElement range_el = doc.createElement("range");
    QRectF rect = this->currentBoundingRect();
    range_el.setAttribute("bottom", QString::number(rect.bottom(), 'f', 6) );
    range_el.setAttribute("top", QString::number(rect.top(), 'f', 6));
    range_el.setAttribute("left", QString::number(rect.left(), 'f', 6));
    range_el.setAttribute("right", QString::number(rect.right() ,'f', 6));
    plot_el.appendChild(range_el);

    QDomElement limitY_el = doc.createElement("limitY");
    if( _custom_Y_limits.min > -MAX_DOUBLE){
        limitY_el.setAttribute("min", QString::number( _custom_Y_limits.min) );
    }
    if( _custom_Y_limits.max < MAX_DOUBLE){
        limitY_el.setAttribute("max", QString::number( _custom_Y_limits.max) );
    }
    plot_el.appendChild(limitY_el);

    for(auto& it: _curve_list)
    {
        auto& name = it.first;
        auto& curve = it.second;
        QDomElement curve_el = doc.createElement("curve");
        curve_el.setAttribute( "name", QString::fromStdString( name ));
        curve_el.setAttribute( "R", curve->pen().color().red());
        curve_el.setAttribute( "G", curve->pen().color().green());
        curve_el.setAttribute( "B", curve->pen().color().blue());
        curve_el.setAttribute( "custom_transform", _curves_transform.at(name) );

        plot_el.appendChild(curve_el);
    }

    QDomElement transform  = doc.createElement("transform");

    if( _action_custom_transform->isChecked() )
    {
        transform.setAttribute("value", tr("Custom::") +_default_transform);
    }
    else{
        transform.setAttribute("value", _default_transform);
    }

    if( _axisX )
    {
        transform.setAttribute("axisX", QString::fromStdString( _axisX->name()) );
    }

    plot_el.appendChild(transform);

    return plot_el;
}

bool PlotWidget::xmlLoadState(QDomElement &plot_widget)
{
    std::set<std::string> added_curve_names;

    QDomElement transform = plot_widget.firstChildElement( "transform" );
    if( !transform.isNull()    )
    {
        if( transform.attribute("value") == "XYPlot")
        {
            QString axisX_name = transform.attribute("axisX");
            if( axisX_name.size()>0){
                changeAxisX( axisX_name );
            }
        }
    }

    QDomElement limitY_el = plot_widget.firstChildElement("limitY");
    if( !limitY_el.isNull() )
    {
        if( limitY_el.hasAttribute("min") ) {
            _custom_Y_limits.min = limitY_el.attribute("min").toDouble();
            _axis_limits_dialog->enableMin( true, _custom_Y_limits.min);
        }
        else{
            _custom_Y_limits.max = -MAX_DOUBLE;
            _axis_limits_dialog->enableMin( false, _custom_Y_limits.min);
        }

        if( limitY_el.hasAttribute("max") ) {
            _custom_Y_limits.max = limitY_el.attribute("max").toDouble();
            _axis_limits_dialog->enableMax( true, _custom_Y_limits.max);
        }
        else{
            _custom_Y_limits.max  = MAX_DOUBLE;
            _axis_limits_dialog->enableMax( false, _custom_Y_limits.max);
        }
    }

    static bool warning_message_shown = false;

    bool curve_added = false;

    for (QDomElement  curve_element = plot_widget.firstChildElement( "curve" )  ;
         !curve_element.isNull();
         curve_element = curve_element.nextSiblingElement( "curve" ) )
    {
        std::string curve_name = curve_element.attribute("name").toStdString();
        int R = curve_element.attribute("R").toInt();
        int G = curve_element.attribute("G").toInt();
        int B = curve_element.attribute("B").toInt();
        QColor color(R,G,B);

        if(  _mapped_data.numeric.find(curve_name) != _mapped_data.numeric.end() )
        {
            auto added = addCurve( curve_name );
            curve_added = curve_added || added;
            _curve_list[curve_name]->setPen( color, 1.0);
            added_curve_names.insert(curve_name );
        }
        else if( ! warning_message_shown )
        {
            QMessageBox::warning(this, "Warning",
                                 tr("Can't find one or more curves.\n"
                                    "This message will be shown only once.") );
            warning_message_shown = true;
        }
    }

    bool curve_removed = true;

    while( curve_removed )
    {
        curve_removed = false;
        for(auto& it: _curve_list)
        {
            auto curve_name = it.first;
            if( added_curve_names.find( curve_name ) == added_curve_names.end())
            {
                removeCurve( curve_name );
                curve_removed = true;
                break;
            }
        }
    }

    if( !transform.isNull()  )
    {
        QString trans_value = transform.attribute("value");
        if( trans_value.isEmpty() ) {
            _action_noTransform->trigger();
        }
        else if( trans_value == Derivative1st )
        {
            _action_1stDerivativeTransform->trigger();
        }
        else if( trans_value == Derivative2nd )
        {
            _action_2ndDerivativeTransform->trigger();
        }
        else if( trans_value == "XYPlot" )
        {
            changeAxisX( transform.attribute("axisX") );
            _action_phaseXY->trigger();
        }
        else if( trans_value.startsWith("Custom::" ) )
        {
            _default_transform = trans_value.remove(0, 8);

            updateAvailableTransformers();

            for (QDomElement  curve_element = plot_widget.firstChildElement( "curve" )  ;
                 !curve_element.isNull();
                 curve_element = curve_element.nextSiblingElement( "curve" ) )
            {
                std::string curve_name = curve_element.attribute("name").toStdString();
                auto custom_attribute = curve_element.attribute("custom_transform");
                if( !custom_attribute.isNull() )
                {
                    _curves_transform[curve_name] = custom_attribute;
                }
            }
            transformCustomCurves();
            _action_custom_transform->setChecked(true);
        }
    }

    if( curve_removed || curve_added)
    {
        replot();
        emit curveListChanged();
    }

    if( curve_removed || curve_added)
    {
        replot();
        emit curveListChanged();
    }

    //-----------------------------------------

    QDomElement rectangle = plot_widget.firstChildElement( "range" );
    if( !rectangle.isNull()){
        QRectF rect;
        rect.setBottom( rectangle.attribute("bottom").toDouble());
        rect.setTop( rectangle.attribute("top").toDouble());
        rect.setLeft( rectangle.attribute("left").toDouble());
        rect.setRight( rectangle.attribute("right").toDouble());
        this->setZoomRectangle( rect, false);
    }

    return true;
}


QRectF PlotWidget::currentBoundingRect() const
{
    QRectF rect;
    rect.setBottom( this->canvasMap( yLeft ).s1() );
    rect.setTop( this->canvasMap( yLeft ).s2() );

    rect.setLeft( this->canvasMap( xBottom ).s1() );
    rect.setRight( this->canvasMap( xBottom ).s2() );

    return rect;
}

void PlotWidget::setZoomRectangle(QRectF rect, bool emit_signal)
{
    this->setAxisScale( yLeft, rect.bottom(), rect.top());
    this->setAxisScale( xBottom, rect.left(), rect.right());

    this->updateAxes();

    if( emit_signal )
    {
        if( isXYPlot()) {
            emit undoableChange();
        }
        else{
            emit rectChanged(this, rect);
        }
    }
}

void PlotWidget::reloadPlotData()
{
    if( isXYPlot() )
    {
        auto it = _mapped_data.numeric.find( _axisX->name() );
        if( it != _mapped_data.numeric.end() ){
            _axisX = &(it->second);
        }
        else{
            _axisX = nullptr;
        }
    }

    for (auto& curve_it: _curve_list)
    {
        auto& curve = curve_it.second;
        const auto& curve_name = curve_it.first;

        auto data_it = _mapped_data.numeric.find( curve_name );
        if( data_it != _mapped_data.numeric.end())
        {
            const auto& data = data_it->second;
            const auto& transform = _curves_transform.at(curve_name);
            auto data_series = createSeriesData( transform, &data);
            curve->setData( data_series );
        }
    }

    if( _curve_list.size() == 0){
        setDefaultRangeX();
    }
}

void PlotWidget::activateLegend(bool activate)
{
//    if( activate ) _legend->attach(this);
//    else           _legend->detach();
    _legend->setVisible(activate);
}

void PlotWidget::activateGrid(bool activate)
{
    _grid->enableX(activate);
    _grid->enableXMin(activate);
    _grid->enableY(activate);
    _grid->enableYMin(activate);
    _grid->attach(this);
}

void PlotWidget::configureTracker(CurveTracker::Parameter val)
{
    _tracker->setParameter( val );
}

void PlotWidget::enableTracker(bool enable)
{
    _tracker->setEnabled( enable && !isXYPlot() );
}

void PlotWidget::setTrackerPosition(double abs_time)
{
    if( isXYPlot()){
        for (auto& it: _curve_list)
        {
            auto& name = it.first;
            auto series = static_cast<DataSeriesBase*>( it.second->data() );
            auto pointXY = series->sampleFromTime(abs_time);
            if( pointXY ){
                _point_marker[name]->setValue( pointXY.value() );
            }
        }
    }
    else{
        double relative_time = abs_time - _time_offset;
        _tracker->setPosition( QPointF( relative_time , 0.0) );
    }
}

void PlotWidget::on_changeTimeOffset(double offset)
{
    _time_offset = offset;
    for(auto& it: _curve_list)
    {
        auto series = static_cast<DataSeriesBase*>( it.second->data() );
        series->setTimeOffset(_time_offset);
    }
    zoomOut(false);
}


PlotData::RangeTime PlotWidget::getMaximumRangeX() const
{
    double left   =  std::numeric_limits<double>::max();
    double right  = -std::numeric_limits<double>::max();

    for(auto& it: _curve_list)
    {
        auto series = static_cast<DataSeriesBase*>( it.second->data() );
        const auto max_range_X = series->getVisualizationRangeX();
        if( !max_range_X ) continue;

        left  = std::min(max_range_X->min, left);
        right = std::max(max_range_X->max, right);
    }

    if( left > right ){
        left  = 0;
        right = 0;
    }

    double margin = 0.0;
    if( fabs(right - left) > std::numeric_limits<double>::epsilon() )
    {
        margin = isXYPlot() ? ((right-left) * 0.025) : 0.0;
    }
    right = right + margin;
    left  = left  - margin;

    return PlotData::RangeTime( {left,right} );
}

//TODO report failure for empty dataset
PlotData::RangeValue  PlotWidget::getMaximumRangeY( PlotData::RangeTime range_X,
                                                    bool absolute_time) const
{
    double top    = -std::numeric_limits<double>::max();
    double bottom =  std::numeric_limits<double>::max();

    for(auto& it: _curve_list)
    {
        auto series = static_cast<DataSeriesBase*>( it.second->data() );

        const auto max_range_X = series->getVisualizationRangeX();
        if( !max_range_X ) continue;

        double left  = std::max(max_range_X->min, range_X.min);
        double right = std::min(max_range_X->max, range_X.max);

        if( !absolute_time )
        {
            left += _time_offset;
            right += _time_offset;
            left = std::nextafter(left, right);
            right = std::nextafter(right, left);
        }

        auto range_Y = series->getVisualizationRangeY( {left, right} );
        if( !range_Y )
        {
            qDebug() << " invalid range_Y in PlotWidget::maximumRangeY";
            continue;
        }
        if( top <    range_Y->max )    top    = range_Y->max;
        if( bottom > range_Y->min )    bottom = range_Y->min;
    }

    double margin = 0.1;

    if( bottom > top ){
        bottom  = 0;
        top = 0;
    }

    if( top - bottom > std::numeric_limits<double>::epsilon() )
    {
        margin = (top-bottom) * 0.025;
    }

    const bool lower_limit = _custom_Y_limits.min > -MAX_DOUBLE;
    const bool upper_limit = _custom_Y_limits.max <  MAX_DOUBLE;

    if(lower_limit)
    {
        bottom = _custom_Y_limits.min;
        if( top < bottom ) top = bottom + margin;
    }

    if( upper_limit )
    {
        top = _custom_Y_limits.max;
        if( top < bottom ) bottom = top - margin;
    }

    if( !lower_limit && !upper_limit )
    {
        top    += margin;
        bottom -= margin;
    }

    return PlotData::RangeValue({ bottom,  top});
}

void PlotWidget::updateCurves()
{
    for(auto& it: _curve_list)
    {
        auto series = static_cast<DataSeriesBase*>( it.second->data() );
        bool res = series->updateCache();
        //TODO check res and do something if false.
    }
}


void PlotWidget::replot()
{
    if( _zoomer )
        _zoomer->setZoomBase( false );

    QwtPlot::replot();
}

void PlotWidget::launchRemoveCurveDialog()
{
    RemoveCurveDialog* dialog = new RemoveCurveDialog(this);
    auto prev_curve_count = _curve_list.size();

    for(auto& it: _curve_list)
    {
        dialog->addCurveName( QString::fromStdString( it.first ),
                              it.second->pen().color() );
    }

    dialog->exec();

    if( prev_curve_count != _curve_list.size() )
    {
        emit undoableChange();
    }
}

void PlotWidget::on_changeColorsDialog_triggered()
{
    std::map<std::string,QColor> color_by_name;

    for(auto& it: _curve_list)
    {
        const auto& curve_name = it.first;
        auto& curve = it.second;
        color_by_name.insert(std::make_pair( curve_name, curve->pen().color() ));
    }

    CurveColorPick* dialog = new CurveColorPick(color_by_name, this);

    connect( dialog, &CurveColorPick::changeColor, this, &PlotWidget::on_changeColor,
             Qt::DirectConnection);

    dialog->exec();

    if( dialog->anyColorModified() )
    {
        emit undoableChange();
    }
}

void PlotWidget::on_changeColor(QString curve_name, QColor new_color)
{
    auto it = _curve_list.find(curve_name.toStdString());
    if( it != _curve_list.end())
    {
        auto& curve = it->second;
        if( curve->pen().color() != new_color)
        {
            curve->setPen( new_color, 1.0 );
        }
        replot();
    }
}

void PlotWidget::on_showPoints_triggered(bool checked)
{
    _show_line_and_points = checked;
    for(auto& it: _curve_list)
    {
        auto& curve = it.second;
        if( _show_line_and_points )
        {
            curve->setStyle( QwtPlotCurve::LinesAndDots);
        }
        else{
            curve->setStyle( QwtPlotCurve::Lines);
        }
    }
    replot();
}

void PlotWidget::on_externallyResized(const QRectF& rect)
{
    if( isXYPlot() )
    {
        emit undoableChange();
    }
    else{
        emit rectChanged(this, rect);
    }
}


void PlotWidget::zoomOut(bool emit_signal)
{
    if( _curve_list.size() == 0)
    {
        QRectF rect(0, 1, 1, -1);
        this->setZoomRectangle(rect, false);
        return;
    }

    QRectF rect;
    auto rangeX = getMaximumRangeX();

    rect.setLeft( rangeX.min );
    rect.setRight( rangeX.max );

    auto rangeY = getMaximumRangeY( rangeX, false );

    rect.setBottom( rangeY.min   );
    rect.setTop(  rangeY.max  );

    _magnifier->setAxisLimits( xBottom, rect.left(),   rect.right() );
    _magnifier->setAxisLimits( yLeft,   rect.bottom(), rect.top() );

    _magnifier->setZoomInKey( Qt::Key_Plus, Qt::ControlModifier);
    _magnifier->setZoomOutKey( Qt::Key_Minus, Qt::ControlModifier);

    this->setZoomRectangle(rect, emit_signal);
}

void PlotWidget::on_zoomOutHorizontal_triggered(bool emit_signal)
{
    QRectF act = currentBoundingRect();
    auto rangeX = getMaximumRangeX();

    act.setLeft( rangeX.min );
    act.setRight( rangeX.max );
    this->setZoomRectangle(act, emit_signal);
}

void PlotWidget::on_zoomOutVertical_triggered(bool emit_signal)
{
    QRectF rect = currentBoundingRect();
    auto rangeY = getMaximumRangeY( {rect.left(), rect.right()}, false );

    rect.setBottom(  rangeY.min );
    rect.setTop(     rangeY.max );

    _magnifier->setAxisLimits( yLeft, rect.bottom(), rect.top() );

    this->setZoomRectangle(rect, emit_signal);
}

void PlotWidget::on_changeToBuiltinTransforms(QString new_transform )
{
    enableTracker(true);

    for(auto& it : _curve_list)
    {
        const auto& curve_name = it.first;
        auto& curve = it.second;

        _point_marker[ curve_name ]->setVisible(false);
        curve->setTitle( QString::fromStdString( curve_name ) );

        if( _curves_transform[curve_name] == new_transform )
        {
            // skip if it is the same
            continue;
        }
        _curves_transform[curve_name] = new_transform;

        auto data_it = _mapped_data.numeric.find( curve_name );
        if( data_it != _mapped_data.numeric.end())
        {
            const auto& data = data_it->second;
            auto data_series = createSeriesData( new_transform, &data);
            curve->setData( data_series );
        }
    }
    _default_transform = new_transform;

    zoomOut(true);
    replot();
}


bool PlotWidget::isXYPlot() const
{
    return _axisX && _action_phaseXY->isChecked();
}


void PlotWidget::on_convertToXY_triggered(bool)
{
    if( !_axisX )
    {
        QMessageBox::warning(this, tr("Warning"),
                             tr("To show a XY plot, you must first provide an alternative X axis.\n"
                                "You can do this drag'n dropping a curve using the RIGHT mouse button "
                                "instead of the left mouse button.") );
        _action_noTransform->trigger();
        return;
    }


    std::deque<PointSeriesXY*> xy_timeseries;

    try{
        for(auto& it: _curve_list)
        {
            const auto& curve_name =  it.first;
            auto& curve =  it.second;
            auto& data = _mapped_data.numeric.find(curve_name)->second;
            xy_timeseries.push_back( new PointSeriesXY( &data, _axisX) );
        }
    }
    catch(std::exception& ex)
    {
        QMessageBox::warning(this, tr("Error"), tr(ex.what()) );
        _action_noTransform->trigger();
        return;
    }

    enableTracker(false);
    _default_transform = "XYPlot";

    for(auto& it: _curve_list)
    {
        const auto& curve_name =  it.first;
        auto& curve =  it.second;
        curve->setData( xy_timeseries.front() );
        xy_timeseries.pop_front();
        _point_marker[ curve_name ]->setVisible(true);
    }

    QFont font_footer;
    font_footer.setPointSize(10);
    QwtText text( QString::fromStdString( _axisX->name()) );
    text.setFont(font_footer);

    this->setFooter( text );

    zoomOut(true);
    replot();
}

void PlotWidget::updateAvailableTransformers()
{
    QSettings settings;
    QByteArray xml_text = settings.value("AddCustomPlotDialog.savedXML",
                                         QByteArray() ).toByteArray();
    if( !xml_text.isEmpty() )
    {
        _snippets = GetSnippetsFromXML(xml_text);
    }
}

void PlotWidget::transformCustomCurves()
{
    std::string error_message;

    for (auto& curve_it: _curve_list)
    {
        auto& curve = curve_it.second;
        const auto& curve_name = curve_it.first;
        const auto& transform = _curves_transform.at(curve_name);

        auto data_it = _mapped_data.numeric.find( curve_name );
        if( data_it != _mapped_data.numeric.end())
        {
            auto& data = data_it->second;
            try {
                auto data_series = createSeriesData( transform, &data);
                curve->setData( data_series );

                if( transform == noTransform || transform.isEmpty())
                {
                    curve->setTitle( QString::fromStdString(curve_name) );
                }
                else{
                    curve->setTitle( QString::fromStdString(curve_name) + tr(" [") + transform +  tr("]") );
                }
            }
            catch (...)
            {
                _curves_transform[curve_name] = noTransform;
                auto data_series = createSeriesData( noTransform, &data);
                curve->setData( data_series );

                error_message += curve_name + (" [") + transform.toStdString() + ("]\n");

                curve->setTitle( QString::fromStdString(curve_name) );
            }
        }
    }
    if( error_message.size() > 0)
    {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("Warnings");
        msgBox.setText(tr("Something went wront while creating the following curves. "
                          "Please check that the transform equation is correct.\n\n") +
                       QString::fromStdString(error_message) );
        msgBox.exec();
    }
}

void PlotWidget::on_customTransformsDialog()
{
    updateAvailableTransformers();

    QStringList available_trans;
    for (const auto& it: _snippets)
    {
        bool valid = true;
        QStringList required_channels = CustomFunction::getChannelsFromFuntion( it.second.equation );
        for (const auto& channel: required_channels)
        {
            if( _mapped_data.numeric.count( channel.toStdString() ) == 0)
            {
                valid = false;
                break;
            }
        }
        valid = valid && it.second.equation.contains("value");

        if( valid )
        {
            available_trans.push_back( it.first );
        }
    }

    TransformSelector dialog( builtin_trans, available_trans,
                              &_default_transform, &_curves_transform,
                              this);

    if (dialog.exec() == QDialog::Rejected )
    {
        return;
    }

    transformCustomCurves();
    zoomOut(false);
    replot();
}

void PlotWidget::changeAxisX(QString curve_name)
{
    auto it = _mapped_data.numeric.find( curve_name.toStdString() );
    if( it != _mapped_data.numeric.end())
    {
        _axisX = &(it->second);
        _action_phaseXY->trigger();
    }
    else{
        //TODO: do nothing (?)
    }
}

void PlotWidget::on_savePlotToFile()
{
    QString fileName;

    QFileDialog saveDialog;
    saveDialog.setAcceptMode(QFileDialog::AcceptSave);
    saveDialog.setDefaultSuffix("png");

    saveDialog.setNameFilter("Compatible formats (*.jpg *.jpeg *.png)");

    saveDialog.exec();

    if(saveDialog.result() == QDialog::Accepted && !saveDialog.selectedFiles().empty())
    {
        fileName = saveDialog.selectedFiles().first();

        QPixmap pixmap (1200,900);
        QPainter * painter = new QPainter(&pixmap);

        if ( !fileName.isEmpty() )
        {
            QwtPlotRenderer rend;
            rend.render(this, painter, QRect(0, 0, pixmap.width(), pixmap.height()));
            pixmap.save(fileName);
        }
    }
}

void PlotWidget::on_editAxisLimits_triggered()
{
    auto rangeX = this->getMaximumRangeX();

    //temporary reset the limit during editing
    _custom_Y_limits.min = -MAX_DOUBLE;
    _custom_Y_limits.max =  MAX_DOUBLE;

    auto rangeY = getMaximumRangeY(rangeX, false);

    _axis_limits_dialog->setDefaultRange(rangeY);
    _axis_limits_dialog->exec();

    _custom_Y_limits = _axis_limits_dialog->rangeY();

    on_zoomOutVertical_triggered(false);
    replot();
    emit undoableChange();
}


bool PlotWidget::eventFilter(QObject *obj, QEvent *event)
{
    switch( event->type() )
    {
    case QEvent::Wheel:
    {
        auto mouse_event = dynamic_cast<QWheelEvent*>(event);
        bool ctrl_modifier = mouse_event->modifiers() == Qt::ControlModifier;
        auto legend_rect = _legend->geometry( canvas()->rect() );

        if ( ctrl_modifier)
        {
            if( legend_rect.contains( mouse_event->pos() )
                && _legend->isVisible() )
            {
                int point_size = _legend->font().pointSize();
                if( mouse_event->delta() > 0 && point_size < 12)
                {
                    emit legendSizeChanged(point_size+1);
                }
                if( mouse_event->delta() < 0 && point_size > 6)
                {
                    emit legendSizeChanged(point_size-1);
                }
                return true; // don't pass to canvas().
            }
        }

        return false;
    }

    case QEvent::MouseButtonPress:
    {
        if( _dragging.mode != DragInfo::NONE)
        {
            return true; // don't pass to canvas().
        }

        QMouseEvent *mouse_event = static_cast<QMouseEvent*>(event);

        if( mouse_event->button() == Qt::LeftButton )
        {
            if( mouse_event->modifiers() == Qt::ShiftModifier) // time tracker
            {
                const QPoint point = mouse_event->pos();
                QPointF pointF ( invTransform( xBottom, point.x()),
                                 invTransform( yLeft, point.y()) );
                emit trackerMoved(pointF);
                return true; // don't pass to canvas().
            }
            else if( mouse_event->modifiers() == Qt::ControlModifier) // panner
            {
                QApplication::setOverrideCursor(QCursor(QPixmap(":/icons/resources/light/move.png")));
            }
            return false; // send to canvas()
        }    
        else if ( mouse_event->buttons() == Qt::MidButton &&
                  mouse_event->modifiers() == Qt::NoModifier )
        {
            QApplication::setOverrideCursor(QCursor(QPixmap(":/icons/resource/lights/move.png")));
            return false;
        }
        else if( mouse_event->button() == Qt::RightButton )
        {
            if( mouse_event->modifiers() == Qt::NoModifier) // show menu
            {
                canvasContextMenuTriggered( mouse_event->pos() );
                return true; // don't pass to canvas().
            }
            else if( mouse_event->modifiers() == Qt::ControlModifier) // Start swapping two plots
            {

                QDrag *drag = new QDrag(this);
                QMimeData *mimeData = new QMimeData;

                QByteArray data;
                QDataStream dataStream(&data, QIODevice::WriteOnly);

                dataStream << this->windowTitle();

                mimeData->setData("plot_area", data );
                drag->setMimeData(mimeData);
                drag->exec();

                return true; // don't pass to canvas().
            }
        }
    }break;
        //---------------------------------
    case QEvent::MouseMove:
    {
        if( _dragging.mode != DragInfo::NONE)
        {
            return true; // don't pass to canvas().
        }

        QMouseEvent *mouse_event = static_cast<QMouseEvent*>(event);

        if ( mouse_event->buttons() == Qt::LeftButton &&
             mouse_event->modifiers() == Qt::ShiftModifier )
        {
            const QPoint point = mouse_event->pos();
            QPointF pointF ( invTransform( xBottom, point.x()),
                             invTransform( yLeft, point.y()) );
            emit trackerMoved(pointF);
            return true;
        }
    }break;

    case QEvent::Leave:
    {
        if( _dragging.mode == DragInfo::NONE )
        {
            changeBackgroundColor( Qt::white );
            QApplication::restoreOverrideCursor();
            return false;
        }
    }break;
    case QEvent::MouseButtonRelease :
    {
        if( _dragging.mode == DragInfo::NONE )
        {
            changeBackgroundColor( Qt::white );
            QApplication::restoreOverrideCursor();
            return false;
        }
    }break;

    case QEvent::Enter:
    {
        // If you think that this code doesn't make sense, you are right.
        // This is the workaround I have eventually found to avoid the problem with spurious
        // QEvent::DragLeave (I have never found the origin of the bug).
        dropEvent(nullptr);
        return true;
    }break;

    default: {}

    } //end switch

    return QWidget::eventFilter( obj, event );
}

void PlotWidget::setDefaultRangeX()
{
    if( _mapped_data.numeric.size() > 0)
    {
        double min =  std::numeric_limits<double>::max();
        double max = -std::numeric_limits<double>::max();
        for (auto& it: _mapped_data.numeric )
        {
            const PlotData& data = it.second;
            if( data.size() > 0){
                double A = data.front().x;
                double B = data.back().x;
                min = std::min( A, min );
                max = std::max( B, max );
            }
        }
        this->setAxisScale( xBottom, min - _time_offset, max - _time_offset);
    }
}

DataSeriesBase *PlotWidget::createSeriesData(const QString &ID, const PlotData *data)
{
    DataSeriesBase *output = nullptr;

    if(ID.isEmpty() || ID == noTransform)
    {
        output = new Timeseries_NoTransform( data );
    }
    else if( ID == Derivative1st || ID == "firstDerivative")
    {
        output = new Timeseries_1stDerivative( data );
    }
    else if( ID == Derivative2nd || ID == "secondDerivative")
    {
        output = new Timeseries_2ndDerivative( data );
    }
    if( ID == "XYPlot")
    {
        try {
            output = new PointSeriesXY( data, _axisX );
        }
        catch (std::runtime_error& ex)
        {
            if( if_xy_plot_failed_show_dialog )
            {
                QMessageBox msgBox(this);
                msgBox.setWindowTitle("Warnings");
                msgBox.setText( tr("The creation of the XY plot failed with the following message:\n %1")
                                .arg( ex.what()) );

//                QAbstractButton* buttonDontRepear = msgBox.addButton("Don't show again",
//                                                                     QMessageBox::ActionRole);
                msgBox.addButton("Continue", QMessageBox::AcceptRole);
                msgBox.exec();

//                if (msgBox.clickedButton() == buttonDontRepear)
//                {
//                    if_xy_plot_failed_show_dialog = false;
//                }
            }
            throw std::runtime_error("Creation of XY plot failed");
        }
    }
    auto custom_it = _snippets.find(ID);
    if( custom_it != _snippets.end())
    {
        const auto& snippet = custom_it->second;
        output = new CustomTimeseries( data, snippet, _mapped_data );
    }

    if( !output ){
        throw std::runtime_error("Not recognized ID in createSeriesData: ");
    }
    output->setTimeOffset( _time_offset );
    return output;
}

void PlotWidget::changeBackgroundColor(QColor color)
{
    if( this->canvasBackground().color() != color)
    {
        this->setCanvasBackground( color );
        this->replot();
    }
}

void PlotWidget::setLegendSize(int size)
{
    auto font = _legend->font();
    font.setPointSize( size );
    _legend->setFont( font );
    replot();
}

bool PlotWidget::isLegendVisible() const
{
    return _legend && _legend->isVisible();
}

void PlotWidget::setLegendAlignment(Qt::Alignment alignment)
{
    _legend->setAlignment( Qt::Alignment( Qt::AlignTop | alignment ) );
}

void PlotWidget::setZoomEnabled(bool enabled)
{
    _zoom_enabled = enabled;
    _zoomer->setEnabled( enabled );
    _magnifier->setEnabled( enabled );
    _panner1->setEnabled( enabled );
    _panner2->setEnabled( enabled );
}

bool PlotWidget::isZoomEnabled() const
{
    return _zoom_enabled;
}

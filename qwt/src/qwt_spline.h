/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_SPLINE_H
#define QWT_SPLINE_H 1

#include "qwt_global.h"
#include "qwt_spline_approximation.h"
#include "qwt_spline_polynomial.h"
#include <qpolygon.h>
#include <qpainterpath.h>
#include <qmath.h>

class QwtSplineParametrization;

/*!
  \brief Base class for a spline interpolation

  Geometric Continuity

    G0: curves are joined
    G1: first derivatives are proportional at the join point
        The curve tangents thus have the same direction, but not necessarily the 
        same magnitude. i.e., C1'(1) = (a,b,c) and C2'(0) = (k*a, k*b, k*c).
    G2: first and second derivatives are proportional at join point 

  Parametric Continuity

    C0: curves are joined
    C1: first derivatives equal
    C2: first and second derivatives are equal

  Geometric continuity requires the geometry to be continuous, while parametric 
  continuity requires that the underlying parameterization be continuous as well.

  Parametric continuity of order n implies geometric continuity of order n, but not vice-versa. 

  QwtSpline is a base class for spline interpolations of any continuity.
*/
class QWT_EXPORT QwtSpline: public QwtSplineApproximation
{
public:
    enum BoundaryPosition
    {
        AtBeginning,
        AtEnd
    };

    enum BoundaryCondition
    {
        Clamped1,

        // Natural := Clamped2 with boundary values: 0.0
        Clamped2,

        // Parabolic runout := Clamped3 with boundary values: 0.0
        Clamped3,

        LinearRunout 
    };

    QwtSpline();
    virtual ~QwtSpline();

    void setBoundaryCondition( BoundaryPosition, int condition );
    int boundaryCondition( BoundaryPosition ) const;

    void setBoundaryValue( BoundaryPosition, double value );
    double boundaryValue( BoundaryPosition ) const;

    void setBoundaryConditions( int condition,
        double valueBegin = 0.0, double valueEnd = 0.0 );

    virtual QPolygonF equidistantPolygon( const QPolygonF &, 
        double distance, bool withNodes ) const;

    virtual QPolygonF polygon( const QPolygonF &, double tolerance );

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual QVector<QLineF> bezierControlLines( const QPolygonF &points ) const = 0;

private:
    Q_DISABLE_COPY(QwtSpline)

    class PrivateData;
    PrivateData *d_data;
};

/*!
  \brief Base class for spline interpolations providing a 
         first order geometric continuity ( G1 ) between adjoing curves
 */
class QWT_EXPORT QwtSplineG1: public QwtSpline
{           
public:     
    QwtSplineG1();
    virtual ~QwtSplineG1();
};

/*!
  \brief Base class for spline interpolations providing a 
         first order parametric continuity ( C1 ) between adjoing curves
 */
class QWT_EXPORT QwtSplineC1: public QwtSplineG1
{
public:
    QwtSplineC1();
    virtual ~QwtSplineC1();

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual QVector<QLineF> bezierControlLines( const QPolygonF & ) const;

    virtual QPolygonF equidistantPolygon( const QPolygonF &,
        double distance, bool withNodes ) const;

    // calculating the parametric equations
    virtual QVector<QwtSplinePolynomial> polynomials( const QPolygonF & ) const;
    virtual QVector<double> slopes( const QPolygonF & ) const = 0;

    // resolving the boundary conditions
    virtual double slopeAtBeginning( const QPolygonF &, double slopeNext ) const;
    virtual double slopeAtEnd( const QPolygonF &, double slopeBefore ) const;
};

/*!
  \brief Base class for spline interpolations providing a 
         second order parametric continuity ( C2 ) between adjoing curves
 */
class QWT_EXPORT QwtSplineC2: public QwtSplineC1
{
public:
    enum BoundaryConditionc2
    {
        // conditions, that require C2 continuity
        CubicRunout = LinearRunout + 1, 
        NotAKnot
    };

    QwtSplineC2();
    virtual ~QwtSplineC2();

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual QVector<QLineF> bezierControlLines( const QPolygonF & ) const;

    virtual QPolygonF equidistantPolygon( const QPolygonF &,
        double distance, bool withNodes ) const;

    // calculating the parametric equations
    virtual QVector<QwtSplinePolynomial> polynomials( const QPolygonF & ) const;
    virtual QVector<double> slopes( const QPolygonF & ) const;
    virtual QVector<double> curvatures( const QPolygonF & ) const = 0;
};

#endif

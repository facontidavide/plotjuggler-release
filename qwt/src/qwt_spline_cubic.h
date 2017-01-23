/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_SPLINE_CUBIC_H
#define QWT_SPLINE_CUBIC_H 1

#include "qwt_global.h"
#include "qwt_spline.h"

class QWT_EXPORT QwtSplineCubic: public QwtSplineC2
{
public:
    QwtSplineCubic();
    virtual ~QwtSplineCubic();

    virtual uint locality() const;

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual QVector<QLineF> bezierControlLines( const QPolygonF &points ) const;

    // calculating the parametric equations
    virtual QVector<QwtSplinePolynomial> polynomials( const QPolygonF & ) const;
    virtual QVector<double> slopes( const QPolygonF & ) const;
    virtual QVector<double> curvatures( const QPolygonF & ) const;

private:
    class PrivateData;
    PrivateData *d_data;
};

#endif

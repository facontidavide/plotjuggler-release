/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_SPLINE_LOCAL_H
#define QWT_SPLINE_LOCAL_H 1

#include "qwt_global.h"
#include "qwt_spline.h"

class QWT_EXPORT QwtSplineLocal: public QwtSplineC1
{
public:
    enum Type
    {
        Cardinal,
        ParabolicBlending, // a.k.a cubic Bessel interpolation
        Akima,
        PChip 
    };

    QwtSplineLocal( Type type );
    virtual ~QwtSplineLocal();

    Type type() const;

    virtual uint locality() const;

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual QVector<QLineF> bezierControlLines( const QPolygonF & ) const;

    // calculating the parametric equations
    virtual QVector<QwtSplinePolynomial> polynomials( const QPolygonF & ) const;
    virtual QVector<double> slopes( const QPolygonF & ) const;

private:
    const Type d_type;
};

#endif

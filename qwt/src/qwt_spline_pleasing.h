/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_SPLINE_PLEASING_H
#define QWT_SPLINE_PLEASING_H 1

#include "qwt_spline.h"

class QWT_EXPORT QwtSplinePleasing: public QwtSplineG1
{
public:
    QwtSplinePleasing();
    virtual ~QwtSplinePleasing();

    virtual uint locality() const;

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual QVector<QLineF> bezierControlLines( const QPolygonF &points ) const;
};

#endif

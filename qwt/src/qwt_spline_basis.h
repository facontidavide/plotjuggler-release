/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_SPLINE_BASIS_H
#define QWT_SPLINE_BASIS_H 1

#include "qwt_global.h"
#include "qwt_spline_approximation.h"

class QWT_EXPORT QwtSplineBasis: public QwtSplineApproximation
{
public:
    QwtSplineBasis();
    virtual ~QwtSplineBasis();

    virtual QPainterPath painterPath( const QPolygonF & ) const;
    virtual uint locality() const;
};

#endif  


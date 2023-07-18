/*
 * Consist, a software for checking map consistency in SLAM
 * Copyright (C) 2013-2014 Mladen Mazuran and Gian Diego Tipaldi and
 * Luciano Spinello and Wolfram Burgard and Cyrill Stachniss
 *
 * This file is part of Consist.
 *
 * Consist is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Consist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Consist.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "quadrilateral.h"
#include "linesegment.h"
#include <assert.h>

namespace consist {

Quadrilateral::Quadrilateral() :
    _vertices(new Point[4])
{
}

Quadrilateral::Quadrilateral(
        const Point &p1v, const Point &p2v, const Point &p3v, const Point &p4v) :
    _vertices(new Point[4])
{
    _vertices[0] = p1v;
    _vertices[1] = p2v;
    _vertices[2] = p3v;
    _vertices[3] = p4v;
}

Quadrilateral::Quadrilateral(const Quadrilateral &q) :
    _vertices(new Point[4])
{
    for(int i = 0; i < 4; i++) {
        _vertices[i] = q._vertices[i];
    }
}

Quadrilateral::~Quadrilateral()
{
    delete _vertices;
}

Quadrilateral &Quadrilateral::operator=(const Quadrilateral &q)
{
    for(int i = 0; i < 4; i++) {
        _vertices[i] = q._vertices[i];
    }
	return *this;
}

bool Quadrilateral::overlapsWith(const Quadrilateral &q) const
{
    if(contains(q.p1()) || contains(q.p2()) || contains(q.p3()) || contains(q.p4())) {
        return true;
    } else if(q.contains(p1()) || q.contains(p2()) || q.contains(p3()) || q.contains(p4())) {
        return true;
    } else {
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                if(linesegment::intersect(
                            _vertices[i], _vertices[(i + 1) % 4],
                            q._vertices[i], q._vertices[(i + 1) % 4])) {
                    return true;
                }
            }
        }
        return false;
    }
}

const Point &Quadrilateral::p1() const
{
    return _vertices[0];
}

const Point &Quadrilateral::p2() const
{
    return _vertices[1];
}

const Point &Quadrilateral::p3() const
{
    return _vertices[2];
}

const Point &Quadrilateral::p4() const
{
    return _vertices[3];
}

Point Quadrilateral::inverseAffine(
        const Point &q01, const Point &q10, const Point &p) const
{
    double den = q01.y() * q10.x() - q01.x() * q10.y();
    double x1 = (q10.x() * p.y() - q10.y() * p.x()) / den; /* New x coordinate */
    double y1 = (q01.y() * p.x() - q01.x() * p.y()) / den; /* New y coordinate */

    return Point(x1, y1);
}

Point Quadrilateral::mapToUnitSquare(const Point &p) const
{
    /*
       Map the (convex) quadrilateral to a quadrilateral with three vertices in (0,0), (0,1), (1,0)
       and then apply a projective transformation to map the last vertex to (1,1).
    */
    Point q01 = p2() - p1();
    Point q10 = p4() - p1();

    Point a  = inverseAffine(q01, q10, p3() - p1());
    Point r1 = inverseAffine(q01, q10, p - p1());

    double a0  = a.x(),  a1  = a.y();
    double r1x = r1.x(), r1y = r1.y();

    /* Apply projective transform */
    double den = a0 * a1 + a1 * (a1 - 1) * r1x + a0 * (a0 - 1) * r1y;
    double x   = (a1 * (a0 + a1 - 1) * r1x) / den;
    double y   = (a0 * (a0 + a1 - 1) * r1y) / den;

    return Point(x, y);
}

bool Quadrilateral::contains(const Point &p) const
{
    Point q = mapToUnitSquare(p);

    return q.x() > 0 && q.x() < 1 && q.y() > 0 && q.y() < 1;
}

double Quadrilateral::area() const
{
    return area(p1(), p2(), p3(), p4());
}

double Quadrilateral::area(const Point &p1, const Point &p2, const Point &p3, const Point &p4)
{
    Point diag1 = p3 - p1, diag2 = p2 - p4;

    return 0.5 * std::abs(diag1.x() * diag2.y() - diag1.y() * diag2.x());
}

double Quadrilateral::rectangleArea(double xmin, double ymin, double xmax, double ymax)
{
    assert(xmax >= xmin && ymax >= ymin);
    return (xmax - xmin) * (ymax - ymin);
}

double Quadrilateral::rectangleOverlapArea(
            double xmin1, double ymin1, double xmax1, double ymax1,
            double xmin2, double ymin2, double xmax2, double ymax2)
{
    assert(xmax1 >= xmin1 && ymax1 >= ymin1);
    assert(xmax2 >= xmin2 && ymax2 >= ymin2);

    double xmin = std::max(xmin1, xmin2);
    double ymin = std::max(ymin1, ymin2);
    double xmax = std::min(xmax1, xmax2);
    double ymax = std::min(ymax1, ymax2);

    if(xmax < xmin || ymax < ymin) {
        return 0;
    } else {
        return rectangleArea(xmin, ymin, xmax, ymax);
    }
}

} /* namespace consist */

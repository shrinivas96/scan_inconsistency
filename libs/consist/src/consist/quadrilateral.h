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

#ifndef QUADRILATERAL_H_
#define QUADRILATERAL_H_

#include "support.h"

namespace consist {

/**
 * \brief Represents an arbitrary convex quadrilateral.
 */
class Quadrilateral
{
public:
    /**
     * \brief Create a quadrilateral with uninitialized vertices.
     */
    Quadrilateral();

    /**
     * \brief Create a quadrilateral and initialize its vertices.
     *
     * The four vertices need to be specified in a connected ordering, e.g.
     * \p p1v is connected to \p p2v, \p p2v is connected to \p p3v, and so
     * on. The constructor doesn't really check if the quadrilateral is
     * convex. It trusts you are one of the good guys.
     *
     * \param p1v First vertex.
     * \param p2v Second vertex.
     * \param p3v Third vertex.
     * \param p4v Fourth vertex.
     */
    Quadrilateral(const Point &p1v, const Point &p2v, const Point &p3v, const Point &p4v);

    /**
     * \brief Copy constructor for Quadrilateral.
     */
    Quadrilateral(const Quadrilateral &q);
    virtual ~Quadrilateral();

    /**
     * \brief Returns the first vertex of the quadrilateral.
     * \return First vertex of the quadrilateral.
     */
    const Point &p1() const;

    /**
     * \brief Returns the second vertex of the quadrilateral.
     * \return Second vertex of the quadrilateral.
     */
    const Point &p2() const;

    /**
     * \brief Returns the third vertex of the quadrilateral.
     * \return Third vertex of the quadrilateral.
     */
    const Point &p3() const;

    /**
     * \brief Returns the fourth vertex of the quadrilateral.
     * \return Fourth vertex of the quadrilateral.
     */
    const Point &p4() const;

    /**
     * \brief Maps a point to the unit square.
     *
     * This method does a projective transformation such that all of the
     * vertices of this quadrilateral are mapped to the vertices of the
     * unit square. This can always be done if the quadrilateral is non
     * degenerate and convex.
     *
     * \param p Point to express in the unit square coordinate system.
     * \return \p p expressed in the unit square coordinate system.
     */
    Point mapToUnitSquare(const Point &p) const;

    /**
     * \brief Checks whether this quadrilateral overlaps/intersects with
     *        \p q
     * \param q Quadrilateral to check against.
     * \return \c true if the two quadrilaterals overlap/intersect,
     *         \c false otherwise.
     */
    bool overlapsWith(const Quadrilateral &q) const;

    /**
     * \brief Checks whether this quadrilateral contains \c p.
     *
     * \param p Query point.
     * \return \c true if this quadrilaterals contains \c p, \c false
     *         otherwise.
     */
    bool contains(const Point &p) const;

    /**
     * \brief Quadrilateral copy operator.
     */
    Quadrilateral &operator=(const Quadrilateral &q);

    /**
     * \brief Returns the area of this quadrilateral.
     * \return Area of this quadrilateral.
     */
    double area() const;

    /**
     * \brief Returns the area of the quadrilateral specified by the vertices.
     *
     * As with the constructor of Quadrilateral, the four vertices need to be
     * specified in a connected ordering, e.g. \p p1 is connected to \p p2,
     * \p p2 is connected to \p p3, and so on. The quadrilateral needs to be
     * convex for this method to return the actual area, although the method
     * doesn't check this.
     *
     * \param p1 First vertex.
     * \param p2 Second vertex.
     * \param p3 Third vertex.
     * \param p4 Fourth vertex.
     * \return Area of the quadrilateral specified by \p p1,\p p2,\p p3,\p p4.
     */
    static double area(const Point &p1, const Point &p2, const Point &p3, const Point &p4);

    /**
     * \brief Returns the area of a non-rotated rectangle.
     *
     * The rectangle is specified as the set of all points \f$(x,y)\f$ such that:
     * \f[
     *      x_{\min} \le x \le x_{\max} \; \wedge \; y_{\min} \le y \le y_{\max}
     * \f]
     *
     * \param xmin Minimum \f$x\f$ value.
     * \param ymin Minimum \f$y\f$ value.
     * \param xmax Maximum \f$x\f$ value.
     * \param ymax Maximum \f$y\f$ value.
     * \return Area of the identified rectangle.
     */
    static double rectangleArea(double xmin, double ymin, double xmax, double ymax);

    /**
     * \brief Returns the area of overlap between a pair of non-rotated
     *        rectangles.
     *
     * The two rectangles are specified in the same format as for rectangleArea().
     * \return Area of the region in which the two quadrilaterals overlap.
     */
    static double rectangleOverlapArea(
            double xmin1, double ymin1, double xmax1, double ymax1,
            double xmin2, double ymin2, double xmax2, double ymax2);

private:
    Point inverseAffine(const Point &q01, const Point &q10, const Point &p) const;

    Point *_vertices;
};

} /* namespace consist */

#endif /* QUADRILATERAL_H_ */

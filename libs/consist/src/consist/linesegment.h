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

#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_

#include "support.h"

namespace consist {

/**
 * \brief Simple functions dealing with line segments.
 */
namespace linesegment {

    /**
     * \brief Minimum distance of a point from a line segment.
     *
     * Computes the minimum distance of a point \p p from the line segment
     * connecting \p q0 and \p q1.
     *
     * \param p  Query point.
     * \param q0 First end point of line segment.
     * \param q1 Second end point of line segment.
     * \return Minimum distance of \p p from [\p q0, \p q1].
     */
    double minimumDistance(const Point &p, const Point &q0, const Point &q1);

    /**
     * \brief Determine whether two line segments intersect.
     *
     * Determines whether the line segment connecting \a p0 and \a p1 and the
     * one connecting \p q0 and \p q1 intersect.
     *
     * \param p0 First end point of first line segment.
     * \param p1 Second end point of first line segment.
     * \param q0 First end point of second line segment.
     * \param q1 Second end point of second line segment.
     * \return \c true if [\p q0, \p q1] and [\p p0, \p p1] intersect,
     *         \p false otherwise.
     */
    bool intersect(
            const Point &p0, const Point &p1, const Point &q0, const Point &q1);

    /**
     * \brief Determine the intersection coefficients of two line segments.
     *
     * Determines the intersection coefficients of the line segment connecting
     * \p p0 and \p p1 and the one connecting \p q0 and \p q1 intersect.
     * By intersection coefficients is meant the pair of values
     * \f$(\lambda,\mu)\f$ solving the following system:
     * \f[
     *  (1-\lambda) \mathbf{p}_0 + \lambda\, \mathbf{p}_1 =
     *  (1-\mu) \mathbf{q}_0 + \mu\, \mathbf{q}_1
     * \f]
     *
     * \param p0 First end point of first line segment.
     * \param p1 Second end point of first line segment.
     * \param q0 First end point of second line segment.
     * \param q1 Second end point of second line segment.
     * \return A vector with the \f$(\lambda,\mu)\f$ entries, in the stated
     *         order.
     */
    Eigen::Vector2d intersectionCoefficients(
            const Point &p0, const Point &p1, const Point &q0, const Point &q1);
} /* namespace linesegment */

} /* namespace consist */

#include "linesegment.hpp"

#endif /* LINESEGMENT_H_ */

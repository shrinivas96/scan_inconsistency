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

#ifndef VISIBILITY_H_
#define VISIBILITY_H_

#include "support.h"
#include "quadrilateral.h"
#include <g2o/core/optimizable_graph.h>

namespace g2o {
    class SE2;
    class RawLaser;
}

namespace consist {

/**
 * \brief An instance of this object describes the visible area perceived in
 *        a 2D laser scan, in the form of a visibility polygon.
 */
class Visibility : public g2o::OptimizableGraph::Data {
public:
    /**
     * \brief Type of frontier for a visibility polygon.
     *
     * This type is used to identify the nature of every edge in a visibility
     * polygon (in the sense of space between two subsequent laser readings).
     */
    enum FrontierType {
        /** The edge is an obstacle. */
        Obstacle,
        /** The edge is free space. */
        FreeSpace,
        /** There is not enough information to determine the nature of the edge. */
        Mistery
    };

    /**
     * \brief Initialize an empty visibility polygon.
     */
    Visibility();

    /**
     * \brief Initialize a visibility polygon from a scan.
     * \param scan Laser scan from which to source the data.
     * \param pose Robot pose from which the laser scan was observed.
     * \param maxRange Maximum acceptable laser range reading. Anything bigger
     *        than this gets truncated to \c maxRange.
     * \param maxDistance Any two subsequent points from a laser scan are
     *        considered as connected by an obstacle if their distance is smaller
     *        than this value, otherwise the space in between is considered free
     *        space.
     */
    Visibility(
            const g2o::RawLaser *scan,
            const g2o::SE2 &pose,
            double maxRange = INFINITY,
            double maxDistance = INFINITY);

    /**
     * \brief Visibility copy constructor.
     */
    Visibility(const Visibility &vis);
    virtual ~Visibility();

    /**
     * \brief Visibility copy operator.
     */
    Visibility &operator=(const Visibility &vis);

    /**
     * \brief Loads object from stream (disabled we don't need it).
     */
    bool read(std::istream& is) { return false; }

    /**
     * \brief Writes object to stream (disabled we don't need it).
     */
    bool write(std::ostream& os) const { return false; }

    /**
     * \brief Computes the inconsistency distances in a visibility polygon
     *        wrt this visibility.
     *
     * This method iterates every vertex of the visibility polygon and checks
     * whether it falls into the polygon of \p other. If it does, then it
     * computes the minimum distance to the boundary of \p other and appends
     * it to the return vector. This method is asymmetric as it does not
     * compute the inconsistency distances in this visibility polygon wrt
     * \p other, but only the converse. The method has quadratic time
     * complexity.
     * \param other Visibility to compare against.
     * \return A \c std::vector containing only the positive inconsistency
     *         distances (i.e. out-of-polygon points are not considered).
     */
    std::vector<double> polyOcclusions(const Visibility &other) const;

    /**
     * \brief Shorthand for points().begin()
     * \see points()
     */
    const PointArray::const_iterator begin() const;

    /**
     * \brief Shorthand for points().end()
     * \see points()
     */
    const PointArray::const_iterator end() const;

    /**
     * \brief Returns the array of vertices of the visibility polygon.
     * \return Array of vertices of the visibility polygon.
     */
    const PointArray &points() const;

    /**
     * \brief Returns the array of laser readings.
     *
     * Actually, if the readings exceed \c maxRange they are truncated to the
     * line connecting the closest preceding and succeeding reading smaller
     * tahn \c maxRange.
     * \return Array of laser readings.
     */
    const PointArray &rays() const;

    /**
     * \brief Returns the point from which the laser scan was taken.
     * \return The point from which the laser scan was taken.
     */
    const Point &viewPoint() const;

    /**
     * \brief Returns the edge type of each edge of the visibility polygon
     * \return \c std::vector containing the edge type of each edge of the
     *         visibility polygon.
     */
    const std::vector<FrontierType> &edgetypes() const;

    /**
     * \brief Checks if the bounding boxes of the visibilities intersect.
     * \param data Query visibility.
     * \return \c true if the bounding box of \p data overlaps/intersects the
     *         bounding box of this visibility polygon, \c false otherwise.
     */
    bool maybeOverlapsWith(const Visibility &data) const;

    /**
     * \brief Checks if this visibility polygon overlaps/intersects with a
     *        quadrilateral.
     * \param q Query quadrilateral.
     * \return \c true if \p q overlaps/intersects this visibility polygon,
     *         \c false otherwise.
     */
    bool overlapsWith(const Quadrilateral &q) const;

    /**
     * \brief Checks if a point is inside the visibility polygon.
     *
     * This method is implemented by checking the number of intersections of
     * a ray connecting \p p and and a point known to be outside the polygon,
     * e.g. \f$(x_{\max}+1,y_{\max}+1)\f$. If the number of intersections is
     * even then the \p p is outside, otherwise it's inside. This is a linear-
     * time algorithm.
     * \param p Query point.
     * \return \c true if \p p is inside this visibility polygon, \c false
     *         otherwise.
     */
    bool inside(const Point &p) const;

    /**
     * \brief Returns the minimum distance from the boundary of the visibility
     *        polygon.
     *
     * Implemented as a linear-time algorithm: the distance from each edge of the
     * visibility polygon is computed and the minimum is taken.
     * \param p Query point.
     * \return minimum distance from the boundary of the visibility polygon.
     */
    double minimumDistance(const Point &p) const;

    /**
     * \brief Rotate and translate the visibility area.
     *
     * Applies the rigid body transformation \p rt to \p vis and returns it.
     * \param rt Rigid body transformation to apply.
     * \param vis Visibility to rotate and translate.
     * \return Rotated and translated \p vis.
     */
    friend Visibility operator*(const g2o::SE2 &rt, const Visibility &vis);

    /**
     * \brief Returns the bounding box of the visibility area.
     * \return The bounding box of the visibility area as a non rotated rectangle.
     */
    Quadrilateral quadrilateral() const;

    /**
     * \brief Returns the minimum x value of bounding box.
     * \return Minimum x value of bounding box.
     */
    double xmin() const { return _xmin; }

    /**
     * \brief Returns the minimum y value of bounding box.
     * \return Minimum y value of bounding box.
     */
    double ymin() const { return _ymin; }

    /**
     * \brief Returns the maximum x value of bounding box.
     * \return Maximum x value of bounding box.
     */
    double xmax() const { return _xmax; }

    /**
     * \brief Returns the maximum y value of bounding box.
     * \return Maximum y value of bounding box.
     */
    double ymax() const { return _ymax; }

protected:
    bool intersects(const Point &p0, const Eigen::Vector2d &to);
    void computePolyline(const g2o::RawLaser *scan, const g2o::SE2 &pose);
    void computeRays(const g2o::RawLaser *scan, const g2o::SE2 &pose);
    void computeBoundingBox();

    Point pointClamp(const Point &from, const Point &to) const;
    bool inRange(double range) const;
    bool closeEnough(const Point &p1, const Point &p2) const;

private:
    PointArray _points, _rays;
    std::vector<FrontierType> _edgetypes;
    Point _viewPoint;
    double _xmin, _xmax;
    double _ymin, _ymax;
    double _maxRange, _maxDistance;
};

} /* namespace consist */

#endif /* VISIBILITY_H_ */

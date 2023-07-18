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

#include "visibility.h"
#include "foreach.h"
#include "linesegment.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <g2o/types/data/raw_laser.h>
#include <g2o/types/slam2d/se2.h>

namespace consist {

Visibility::Visibility() {
}

Visibility::Visibility(
        const g2o::RawLaser *scan, const g2o::SE2 &pose,
        double maxRange, double maxDistance) :
        _viewPoint(pose.translation()),
        _maxRange(maxRange), _maxDistance(maxDistance)
{
    computePolyline(scan, pose);
    computeRays(scan, pose);
    computeBoundingBox();
}

Visibility::Visibility(const Visibility &vis)
{
    *this = vis;
}

Visibility::~Visibility() {
    // TODO Auto-generated destructor stub
}

Visibility &Visibility::operator=(const Visibility &vis)
{
    _points      = vis._points;
    _rays        = vis._rays;
    _edgetypes   = vis._edgetypes;
    _viewPoint   = vis._viewPoint;
    _xmin        = vis._xmin;
    _xmax        = vis._xmax;
    _ymin        = vis._ymin;
    _ymax        = vis._ymax;
    _maxRange    = vis._maxRange;
    _maxDistance = vis._maxDistance;
    return *this;
}

const PointArray &Visibility::points() const
{
    return _points;
}

const Point &Visibility::viewPoint() const
{
    return _viewPoint;
}

const PointArray &Visibility::rays() const
{
    return _rays;
}

const std::vector<Visibility::FrontierType> &Visibility::edgetypes() const
{
    return _edgetypes;
}

Point Visibility::pointClamp(const Point &from, const Point &to) const
{
    if((to - from).norm() < _maxRange) {
        return to;
    } else {
        return from + _maxRange * (to - from).normalized();
    }
}

bool Visibility::inRange(double range) const
{
    return range <= _maxRange && range > 0;
}

bool Visibility::closeEnough(const Point &p1, const Point &p2) const
{
    return (p1 - p2).norm() <= _maxDistance;
}

void Visibility::computePolyline(const g2o::RawLaser *scan, const g2o::SE2 &pose) {
    size_t i = 0;
    const std::vector<double> &ranges = scan->ranges();
    PointArray pts;

    for(int i = 0; i < ranges.size(); i++) {
        double t = scan->laserParams().firstBeamAngle + i * scan->laserParams().fov / (ranges.size() - 1); // i * scan->laserParams().angularStep;
        pts.push_back(pose * Point(ranges[i] * std::cos(t), ranges[i] * std::sin(t)));
    }

    while(i < pts.size()) {
        _points.push_back(pointClamp(_viewPoint, pts[i]));

        if(i + 1 < pts.size()) {
            if(inRange(ranges[i]) && inRange(ranges[i + 1])) {
                if(closeEnough(_points.back(), pointClamp(_viewPoint, pts[i + 1]))) {
                    _edgetypes.push_back(Visibility::Obstacle);
                } else {
                    _edgetypes.push_back(Visibility::Mistery);
                }
            } else {
                /* skip subsequent max-range values */
                while(i + 2 < pts.size() && !inRange(ranges[i + 1])) {
                    i++;
                }
                _edgetypes.push_back(Visibility::FreeSpace);
            }
        }
        i++;
    }

}

void Visibility::computeRays(const g2o::RawLaser *scan, const g2o::SE2 &pose) {
    const std::vector<double> &ranges = scan->ranges();
    _rays.push_back(_points[0]);

    for(size_t i = 1, j = 1; i < ranges.size() - 1; i++) {
        if(inRange(ranges[i])) {
            _rays.push_back(_points[j++]);
        } else {
            double t = scan->laserParams().firstBeamAngle + i * scan->laserParams().angularStep;
            Point direction = pose * Point(std::cos(t), std::sin(t));

            Eigen::Vector2d l12 = linesegment::intersectionCoefficients(
                    _viewPoint, direction, _points[j - 1], _points[j]);
            _rays.push_back((1 - l12[0]) * _viewPoint + l12[0] * direction);
        }
    }
    _rays.push_back(_points.back());
}

const PointArray::const_iterator Visibility::begin() const {
    return _points.begin();
}

const PointArray::const_iterator Visibility::end() const {
    return _points.end();
}

std::vector<double> Visibility::polyOcclusions(const Visibility &other) const
{
    std::vector<double> occlusions;
    fforeach(const Point &p, _points) {
        /*
         * First check the bounding box as it's very cheap (constant time).
         * Only if the bounding box test passes check if p occludes the free
         * space of other, as it's expensive (linear time).
         */
        if(p.x() > other.xmin() && p.x() < other.xmax() &&
                p.y() > other.ymin() && p.y() < other.ymax() &&
                (p - _viewPoint).norm() <= _maxRange - 1e-5 &&
                other.inside(p)) {
            double d = other.minimumDistance(p);
            occlusions.push_back(d);
        }
    }
    return occlusions;
}

void Visibility::computeBoundingBox() {
    _xmin = _xmax = _viewPoint.x();
    _ymin = _ymax = _viewPoint.y();
    fforeach(const Point &p, _rays) {
        _xmin = std::min(_xmin, p.x());
        _xmax = std::max(_xmax, p.x());
        _ymin = std::min(_ymin, p.y());
        _ymax = std::max(_ymax, p.y());
    }
}

static bool between(double z, double zmin, double zmax) {
    return z >= zmin && z <= zmax;
}

bool Visibility::maybeOverlapsWith(const Visibility &other) const {
    return (between(_xmin, other._xmin, other._xmax) ||
            between(other._xmin, _xmin, _xmax)) &&
            (between(_ymin, other._ymin, other._ymax) ||
             between(other._ymin, _ymin, _ymax));
}

bool Visibility::overlapsWith(const Quadrilateral &q) const
{
    const Point vertices[] = {
        q.p1(), q.p2(), q.p3(), q.p4()
    };

    for(size_t i = 0; i < support::arraySize(vertices); i++) {
        const Point &q1 = vertices[i], &q2 = vertices[(i + 1) % support::arraySize(vertices)];
        for(size_t j = 0; j < _edgetypes.size(); j++) {
            if(linesegment::intersect(q1, q2, _points[j], _points[j + 1]))
                return true;
        }
        if(linesegment::intersect(q1, q2, _viewPoint, _points.front()))
            return true;
        if(linesegment::intersect(q1, q2, _viewPoint, _points.back()))
            return true;
    }

    return (q.contains(_viewPoint)) || inside(vertices[0]);
}

bool Visibility::inside(const Point &p) const {
    const Point p2(_xmax + 1, _ymax + 1);
    int count = 0;

    for(size_t i = 0; i < _edgetypes.size(); i++) {
        if(linesegment::intersect(p, p2, _points[i], _points[i + 1])) count++;
    }

    if(linesegment::intersect(p, p2, _viewPoint, _points.front())) count++;
    if(linesegment::intersect(p, p2, _viewPoint, _points.back())) count++;

    return count % 2 == 1;
}

double Visibility::minimumDistance(const Point &p) const {
    double mindist = INFINITY;
    for(size_t i = 0; i < _edgetypes.size(); i++) {
        mindist = std::min(mindist, linesegment::minimumDistance(p, _points[i], _points[i + 1]));
    }
    mindist = std::min(mindist, linesegment::minimumDistance(p, _viewPoint, _points.front()));
    mindist = std::min(mindist, linesegment::minimumDistance(p, _viewPoint, _points.back()));
    return mindist;
}

Quadrilateral Visibility::quadrilateral() const
{
    return Quadrilateral(Point(_xmin, _ymin), Point(_xmax, _ymin), Point(_xmax, _ymax), Point(_xmin, _ymax));
}

Visibility operator*(const g2o::SE2 &rt, const Visibility &vis) {
    Visibility ret;

    fforeach(const Point &point, vis._points) {
        ret._points.push_back(rt * point);
    }

    fforeach(const Point &ray, vis._rays) {
        ret._rays.push_back(rt * ray);
    }

    ret._edgetypes = vis._edgetypes;
    ret._viewPoint = rt * vis._viewPoint;
    ret.computeBoundingBox();

    return ret;
}

} /* namespace consist */

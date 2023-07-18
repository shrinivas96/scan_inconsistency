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

namespace consist {

namespace linesegment {

inline double minimumDistance(const Point &p, const Point &q0, const Point &q1)
{
#if 0
    const Point direction = (q1 - q0).normalized();
    const Point orthoDirection(- direction.y(), direction.x());
    const Eigen::Vector2d l12 = intersectionCoefficients(q0, q1, p, p + orthoDirection);
    if(l12[0] >= 0 && l12[0] <= 1) {
        const Point intercept = (1 - l12[0]) * q0 + l12[0] * q1;
        return (p - intercept).norm();
    } else {
        return std::min((p - q0).norm(), (p - q1).norm());
    }
#else
    const Point pc = p - q0, qc = q1 - q0;
    const double qcnorm = qc.norm();
    const double proj   = pc.dot(qc) / qcnorm;
    if(proj < 0 || proj > qcnorm) {
        return std::min(pc.norm(), (p - q1).norm());
    } else {
        return std::abs(pc.x() * qc.y() - pc.y() * qc.x()) / qcnorm;
    }
#endif
}

inline bool intersect(
        const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    Eigen::Vector2d l12 = intersectionCoefficients(p0, p1, q0, q1);
    return l12[0] >= 0 && l12[0] <= 1 && l12[1] >= 0 && l12[1] <= 1;
}

inline Eigen::Vector2d intersectionCoefficients(
        const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    Eigen::Vector2d ret;
    const double tmp0 = p0.y() - p1.y();
    const double tmp1 = q0.x() - q1.x();
    const double tmp2 = p0.x() - p1.x();
    const double tmp3 = q0.y() - q1.y();
    const double den  = tmp2 * tmp3 - tmp0 * tmp1;
    ret[0] = (-q0.y() * q1.x() - p0.y() * tmp1 + q0.x() * q1.y() + p0.x() * tmp3) / den;
    ret[1] = -(-p1.y() * q0.x() + p0.y() * (q0.x() - p1.x()) + p0.x() *
               (p1.y() - q0.y()) + p1.x() * q0.y()) / den;
    return ret;
}

} /* namespace linesegment */

} /* namespace consist */

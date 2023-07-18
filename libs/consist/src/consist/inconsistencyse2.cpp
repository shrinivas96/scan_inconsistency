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

#include "inconsistencyse2.h"
#include "quadrilateral.h"

namespace consist {

InconsistencySE2::InconsistencySE2()
{
}

InconsistencySE2::InconsistencySE2(const Quadrilateral &region, InconsistencyType type) :
        _region(region), _type(type)
{
}

InconsistencySE2::~InconsistencySE2()
{
}

bool InconsistencySE2::read(std::istream& is)
{
    int type;
    Point p1, p2, p3, p4;
    is >> type >>
            p1.x() >> p1.y() >>
            p2.x() >> p2.y() >>
            p3.x() >> p3.y() >>
            p4.x() >> p4.y();
    _region = Quadrilateral(p1, p2, p3, p4);
    _type   = (InconsistencyType) type;
    return true;
}

bool InconsistencySE2::write(std::ostream& os) const
{
    os << (int) _type << " " <<
            _region.p1().x() << " " << _region.p1().y() << " " <<
            _region.p2().x() << " " << _region.p2().y() << " " <<
            _region.p3().x() << " " << _region.p3().y() << " " <<
            _region.p4().x() << " " << _region.p4().y() << " ";
    return os.good();
}

const Quadrilateral &InconsistencySE2::region() const
{
    return _region;
}

void InconsistencySE2::setRegion(const Quadrilateral &q)
{
    _region = q;
}

InconsistencySE2::InconsistencyType InconsistencySE2::type() const
{
    return _type;
}

void InconsistencySE2::setType(InconsistencyType t)
{
    _type = t;
}


} /* namespace consist */

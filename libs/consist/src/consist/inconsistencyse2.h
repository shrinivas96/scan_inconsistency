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

#ifndef INCONSISTENCYSE2_H_
#define INCONSISTENCYSE2_H_

#include <iostream>
#include "quadrilateral.h"

namespace consist {

/**
 * \brief Class representing a tagged inconsistency on a 2D map.
 *
 * Tagged inconsistencies are represented as arbitrary convex quadrilaterals.
 * We make a distinction between possible typed of inconsistencies. We refer
 * to as "strong" inconsistencies those which involve different regions of
 * the map incorrectly overlapping. On the other hand, we refer to as "weak"
 * inconsistencies those resulting from an unnatural incremental bending of
 * a local map area.
 */
class InconsistencySE2
{
public:

    /**
     * \brief Inconsistency type, see InconsistencySE2 description for further
     *        details.
     * \see InconsistencySE2
     */
    enum InconsistencyType {
        /** \brief Strong inconsistency. */
        Strong = 0,
        /** \brief Weak inconsistency. */
        Weak = 1
    };

    /**
     * \brief Create an uninitialized inconsistency.
     *
     * The quadrilateral region and inconsistency type are left uninitialized.
     */
    InconsistencySE2();

    /**
     * \brief Create and initialize an inconsistency.
     *
     * \param region Quadrilateral describing the rough area where the
     *               inconsistency appears.
     * \param type Inconsistency type.
     */
    InconsistencySE2(const Quadrilateral &region, InconsistencyType type);
    virtual ~InconsistencySE2();

    /**
     * \brief Load an inconsistency from a stream.
     *
     * \param is Stream from which the inconsistency region and type will be
     *           read.
     * \return \c true on success, \c false otherwise.
     */
    bool read(std::istream& is);

    /**
     * \brief Write an inconsistency to a stream.
     *
     * \param os Stream to which the inconsistency region and type will be
     *           written.
     * \return \c true on success, \c false otherwise.
     */
    bool write(std::ostream& os) const;

    /**
     * \brief Return the rough inconsistent region.
     * \return The region in the form of a Quadrilateral.
     */
    const Quadrilateral &region() const;

    /**
     * \brief Set the rough inconsistent region.
     * \param q The region in the form of a Quadrilateral.
     */
    void setRegion(const Quadrilateral &q);

    /**
     * \brief Return the inconsistency type.
     * \return Inconsistency type.
     */
    InconsistencyType type() const;

    /**
     * \brief Set the inconsistency type.
     * \param t Inconsistency type.
     */
    void setType(InconsistencyType t);

private:
    InconsistencyType _type;
    Quadrilateral _region;
};

} /* namespace consist */
#endif /* INCONSISTENCYSE2_H_ */

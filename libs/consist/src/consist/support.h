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

#ifndef SUPPORT_H_
#define SUPPORT_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <vector>
#include <iostream>
#include <g2o/core/optimizable_graph.h>
#include <g2o/types/slam2d/se2.h>
#include "foreach.h"

namespace consist {

/** \brief Define point as a 2D vector. */
typedef Eigen::Vector2d Point;
/** \brief Define an array of points as a \c std::vector of Point objects. */
typedef std::vector<Point, Eigen::aligned_allocator<Point> > PointArray;

/**
 * \brief Rotates and translates an array of points.
 * \param rt Rigid body transformation to apply.
 * \param list Array of points to rotate.
 * \return \p list rotated and translated by \p rt.
 */
PointArray operator*(const g2o::SE2 &rt, const PointArray &list);

/**
 * \brief Prints a point to a stream.
 *
 * Formats the point in the form <tt>{p.x(),p.y()}</tt>.
 *
 * \param stream Stream to print to.
 * \param p Point to print.
 * \return The modified stream.
 */
std::ostream &operator<<(std::ostream &stream, const Point &p);

/**
 * \brief Prints a \c std::vector of elements to a stream.
 *
 * Formats the \c std::vector in the form <tt>{vec[0],...,vec[n]}</tt>.
 *
 * \param stream Stream to print to.
 * \param vec \c std::vector to print.
 * \return The modified stream.
 */
template <typename T, typename A>
std::ostream &operator<<(std::ostream &stream, const std::vector<T, A> &vec);

/** \brief Miscellaneous support functions */
namespace support {

/**
 * \brief Hides the cursor on a console stream.
 *
 * On a standard UNIX virtual terminal this simply prints the escape
 * sequence <tt>"\e[?25l"</tt>.
 *
 * \param stream Stream to print to.
 * \return The modified stream.
 */
std::ostream &cursorHide(std::ostream &stream);

/**
 * \brief Shows the cursor on a console stream.
 *
 * On a standard UNIX virtual terminal this simply prints the escape
 * sequence <tt>"\e[?25h"</tt>.
 *
 * \param stream Stream to print to.
 * \return The modified stream.
 */
std::ostream &cursorShow(std::ostream &stream);

/**
 * \brief Finds the first datum of the specified type
 *
 * This function casts \p v to a \c g2o::OptimizableGraph::Vertex and
 * searches for the first object in the user data of the vertex which
 * is of the specified type.
 *
 * \tparam T Type of the datum to search for.
 * \param v Vertex to search for data.
 * \return The first datum of type \p T attached to \p v. If either
 *         \p v is not an instance of \c g2o::OptimizableGraph::Vertex
 *         or \p v does not have any user data of type \p T then this
 *         function return NULL.
 */
template <typename T>
T *findFirstDatum(g2o::HyperGraph::Vertex *v);

/**
 * \brief Finds the first datum of the specified type
 *
 * This is a const overload of findFirstDatum(g2o::HyperGraph::Vertex *)
 * \see findFirstDatum(g2o::HyperGraph::Vertex *)
 */
template <typename T>
const T *findFirstDatum(const g2o::HyperGraph::Vertex *v);

/**
 * \brief Computes the sum of the values in a container.
 *
 * \param c The container to consider. Needs to implement \c begin(),
 *          \c end() and requires a type \c value_type to be defined.
 *          Furthermore \c value_type needs to support elementwise
 *          addition.
 * \return The sum of all elements in \p c.
 */
template <typename Container>
typename Container::value_type sum(const Container &c);

/**
 * \brief Computes the mean of the values in a container.
 *
 * \param c The container to consider. Needs to implement \c begin(),
 *          \c end() and requires a type \c value_type to be defined.
 *          Furthermore \c value_type needs to support elementwise
 *          addition and division by \c int.
 * \return The mean of all elements in \p c. If \p c has no elements
 *         the function returns 0.
 */
template <typename Container>
typename Container::value_type mean(const Container &c);

/**
 * \brief Computes the variance of the values in a container.
 *
 * \param c The container to consider. Needs to implement \c begin(),
 *          \c end() and requires a type \c value_type to be defined.
 *          Furthermore \c value_type needs to support elementwise
 *          addition, elementwise multiplication and division by \c int.
 * \return The variance of all elements in \p c. If \p c has less than
 *         two elements the function returns 0.
 */
template <typename Container>
typename Container::value_type variance(const Container &c);

/**
 * \brief Helper function to determine the compile time size of an array.
 *
 * \return The compile time size of the unnamed array parameter.
 */
template <typename T, size_t N>
size_t arraySize(const T (&)[N]) { return N; }

} /* namespace support */

} /* namespace consist */

#include "support.hpp"

#endif /* SUPPORT_H_ */

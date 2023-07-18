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

#ifndef MAPQUALITY_H_
#define MAPQUALITY_H_

#include <Eigen/Core>
#include "visibility.h"
#include "mapqualityoptions.h"

namespace g2o {
    class SparseOptimizer;
    class RawLaser;
    class VertexSE2;
}

namespace consist {

/**
 * \brief A functor that evaluates the quality of a 2D map.
 *
 * An instance of this class behaves as a functor that assigns a scalar value
 * to a map describing its level of consistency. The map input is taken as a
 * \c g2o::SparseOptimizer with \c g2o::VertexSE2 vertices and associated
 * \c g2o::RawLaser data objects.
 * This functor is to be used as an argument to GridBisectionOptimizer for
 * determining the optimal \f$\Phi\f$ parameter of DCS.
 */
class MapQuality {
public:
    /**
     * \brief Initialize a new functor.
     */
    MapQuality();

    /**
     * \brief Initialize a new functor and assign a map.
     * \param so Map to evaluate.
     * \see setMap
     */
    MapQuality(g2o::SparseOptimizer *so);
    virtual ~MapQuality();

    /**
     * \brief Assign map to evaluate.
     *
     * Take note of the fact that \p so needs to be non-const. This is because
     * operator()(double) re-optimizes the map, and thus needs to be able to modify
     * \p so. Nevertheless operator()(double) will restore the state of \p so
     * prior to the method's invocation.
     *
     * \param so Map to evaluate.
     * \see operator()()
     * \see operator()(double)
     */
    void setMap(g2o::SparseOptimizer *so);

    /**
     * \brief Sets the options for this MapQuality object.
     * \param opts Options to set.
     */
    void setOptions(const MapQualityOptions &opts);

    /**
     * \brief Returns the current options for this MapQuality object.
     * \return Current options.
     */
    const MapQualityOptions &options() const;

    /**
     * \brief Evaluate the quality of the assigned map.
     *
     * If no map has been assigned this method will quietly return +inf. Note that
     * this method does not optimize the map, it only takes in the current
     * linearization point and evaluates the quality of the map given said
     * linearization point.
     *
     * \return Quality of the map.
     */
    double operator()();

    /**
     * \brief Evaluate the quality of the assigned map for given \f$\Phi\f$.
     *
     * If no map has been assigned this method will quietly return +inf.
     * If the the edges (only g2o::EdgeSE2 are considered, currently) have already
     * a robust kernel assigned this method will change their \p delta to \p phi.
     * If no robust kerner is assigned, this method will create a new DCS kernel
     * with parameter \p phi. The map is then optimized and fed to operator()().
     * Note that the complete state of the map is restored prior to
     * operator()(double phi) returning.
     *
     * \param phi Scale parameter for the DCS and/or other robust kernel.
     * \return Quality of the map.
     */
    double operator()(double phi);

    /**
     * \brief Returns how many times operator()(double) has been called since the
     *        last reset.
     * \return Number of evaluations.
     */
    int iterationCount() const;

    /**
     * \brief Resets the number of functional evaluations to 0.
     */
    void resetIterationCount();

private:
    static void rowThread(int index, MapQuality *q);
    double computeMeasure();

private:
    g2o::SparseOptimizer *_so;
    MapQualityOptions _opts;
    Eigen::MatrixXd _pairwise;
    std::vector<Visibility> _visibilities;
    int _iteration;
    double _timing;
};

} /* namespace consist */

#endif /* MAPQUALITY_H_ */

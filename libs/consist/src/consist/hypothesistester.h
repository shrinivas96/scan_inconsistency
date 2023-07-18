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

#ifndef HYPOTHESISTESTER_H_
#define HYPOTHESISTESTER_H_

#include "hypothesistesteroptions.h"
#include <Eigen/Core>
#include <vector>

namespace g2o { class SparseOptimizer; }

namespace consist {

class Visibility;

/**
 * \brief Map consistency evaluator
 *
 * An instance of this class allows to evaluate the consistency of a 2D map
 * with \c g2o::RawLaser objects attached as user data to \c g2o::VertexSE2
 * vertices in a g2o::SparseOptimizer.
 *
 * A sample usage code is:
 * \code
 * HypothesisTester ht;
 * ht.setMap(someMap);
 * ht.setOptions(someOptions); // Optional
 * ht.computeConsistencyMatrix();
 * std::vector<int> inconsistentIds = ht.binomialHypothesisTest();
 * \endcode
 */
class HypothesisTester {
public:
    /**
     * \brief Initialized a new HypothesisTester with the default options.
     */
    HypothesisTester();

    /**
     * \brief Automatically deallocates the full and incremental clone maps
     *        if they have been allocated.
     */
    virtual ~HypothesisTester();

    /**
     * \brief Sets the options for this HypothesisTester object.
     * \param opts Options to set.
     */
    void setOptions(const HypothesisTesterOptions &opts);

    /**
     * \brief Returns the current options for this HypothesisTester object.
     * \return Current options.
     */
    const HypothesisTesterOptions &options() const;

    /**
     * \brief Sets the map for which to evaluate the consistency.
     *
     * This method will clone the \p so map in both its incremental
     * odometry-only version and its full version.
     * The map \p so will be left unchanged.
     */
    void setMap(const g2o::SparseOptimizer *so);

    /**
     * \brief Returns the cloned full map.
     *
     * The full map is generated once setMap() has been called, if
     * setMap() has not been called before this method will return NULL.
     *
     * \return Full map clone.
     */
    g2o::SparseOptimizer *fullMap();

    /**
     * \brief Returns the cloned incremental map.
     *
     * The incremental map is generated once setMap() has been called, if
     * setMap() has not been called before this method will return NULL.
     *
     * \return Incremental map clone.
     */
    g2o::SparseOptimizer *incrementalMap();

    /**
     * \brief Computes the pairwise consistency matrix for the scans of a map.
     *
     * Requires setMap() to have been called, otherwise the behavior of
     * this method is undefined.
     */
    void computeConsistencyMatrix();

    /**
     * \brief Retrieve the pairwise consistency matrix.
     *
     * Assumes that computeConsistencyMatrix() has already been called, the
     * behavior of the method is undefined otherwise.
     *
     * \return The computed pairwise consistency matrix.
     */
    const Eigen::MatrixXd &consistencyMatrix() const;

    /**
     * \brief Carries out a binomial hypothesis test on the consistency
     *        matrix.
     * \param simpleFiltering If \c true, this will enable a very
     *        rudimentary filtering of false positives (a positive is kept
     *        only if there is another positive in a locality of 4 scans
     *        forward and 4 scans backward).
     * \return A \c std::vector containing the indices of the vertices
     *         associated to the scans which appear inconsistent according
     *         to the binomial hypothesis test.
     */
    std::vector<int> binomialHypothesisTest(bool simpleFiltering = true);

protected:
    /**
     * \brief Clones a SparseOptimizer and all its vertices/edges.
     *
     * \param so Graph to clone.
     * \param incrementalOnly \c false to clone the full graph, \c true
     *                        to clone only the odometry chain.
     * \return The cloned graph.
     */
    g2o::SparseOptimizer *clone(
            const g2o::SparseOptimizer *so, bool incrementalOnly = false);

    /**
     * \brief Adds Visibility data objects to \p so.
     *
     * For each vertex in \p so which has an associated \c g2o::RawLaser this
     * method adds a further Visibility data object to the vertex.
     *
     * \param so Graph to modify.
     * \return A \c std::vector containing all of the data objects added to
     *         the graph.
     */
    std::vector<Visibility *> addVisibilities(g2o::SparseOptimizer *so);

    /**
     * \brief The progress function which is fed to ParallelRunner.
     *
     * As of now this just prints \"\p index/\p max (\p STAGE)\\r\".
     */
    template <int STAGE>
    static void progressDisplayer(int index, int max, HypothesisTester *ht);

    /**
     * \brief Thread function for the evaluation of the incremental
     *        parameters.
     */
    static void incrementalParametersThread(int index, HypothesisTester *ht);

    /**
     * \brief Thread function for the evaluation of the pairwise consistencies.
     */
    static void consistencyMatrixThread(int index, HypothesisTester *ht);

private:
    HypothesisTesterOptions _opts;
    g2o::SparseOptimizer *_full, *_incremental;
    std::vector<Visibility *> _visibilitiesFull, _visibilitiesIncremental;
    Eigen::MatrixXd _consistency;
    Eigen::VectorXd _means, _variances;
};

} /* namespace consist */

#endif /* HYPOTHESISTESTER_H_ */

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

#ifndef HYPOTHESISTESTEROPTIONS_H_
#define HYPOTHESISTESTEROPTIONS_H_

namespace consist {

/**
 * \brief Evaluation options for HypothesisTester
 * \see HypothesisTester
 */
struct HypothesisTesterOptions {
    /**
     * \brief Maximum acceptable laser range reading.
     *
     * Anything bigger than this gets truncated to \c maxRange. The default
     * value is 8 meters.
     */
    double maxRange;

    /**
     * \brief Maximum obstacle detection distance.
     *
     * Any two subsequent points from a laser scan are considered as connected
     * by an obstacle if their distance is smaller than this value, otherwise
     * the space in between is considered free space. The default value is
     * 0.2 meters.
     */
    double maxDistance;

    /**
     * \brief Minimum node spacing between subsequent laser scans.
     *
     * Effectively decimates the input scans, the spacing is considered in terms
     * of order of appearence in the dataset. Note that 1 accepts all laser
     * scans. The default is 1.
     */
    int nodeSpacing;

    /**
     * \brief Incremental map locality radius.
     *
     * How many nodes to look forwards/backwards in the incremental only map for the
     * distribution parameters estimation procedure. The default value is 20.
     */
    int locality;

    /**
     * \brief How many threads to use for the computation.
     *
     * If 0 the number of threads is chosen automatically. The default value is 0.
     */
    int threads;

    /**
     * \brief Be verbose when computing the consistency.
     *
     * Currently this only displays the progress on stderr. This is enabled by
     * default.
     */
    bool verbose;


    /**
     * \brief \f$1 - \alpha\f$ for the binomial hypothesis test.
     *
     * The default value is 0.01
     */
    double binomialError;

    /**
     * \brief Probability of failure of the normal hypothesis test.
     *
     * This is fed into the binomial test. Ideally, this should be equivalent to
     * HypothesisTesterOptions::normalError, in reality this value should be
     * slightly larger in order to account for the error introduced by
     * approximating a finite sum of random variables with a normal random
     * variable. The default value is 0.015.
     */
    double binomialProbability;

    /**
     * \brief \f$1 - \alpha\f$ for a single normal hypothesis test.
     *
     * The default value is 0.01
     */
    double normalError;

    /**
     * \brief \f$1 - \alpha\f$ for estimating the distribution parameters.
     *
     * This value should be larger than HypothesisTesterOptions::normalError to
     * account for the fact that random samples from a normal distribution do not
     * necessarily fall at the maximum allowed error. Choosing a smaller or equal
     * value would result in an overly optimistic estimate.
     * The default value is 0.05
     */
    double incrementalNormalError;

    /**
     * \brief Minimum number of free space occlusions
     *
     * If two scans do not occlude each other's free space by more than this
     * number of instances then they are not considered as overlapping.
     * This is for robustness reasons. The default value is 10.
     */
    int minimumOcclusionCount;

    /**
     * \brief Number of occlusions to consider as outliers.
     *
     * This value specifies that the top n inconsistencies when computing the
     * inconsistency distances between two scans should be dropped. This is for
     * robustness reasons. The default value is 10.
     */
    int occlusionOutlierCount;

    /**
     * \brief Initializes a new object with the default options.
     */
    HypothesisTesterOptions() :
        maxRange(8),
        maxDistance(.2),
        nodeSpacing(1),
        locality(20),
        threads(0),
        verbose(true),
        binomialError(0.01),
        binomialProbability(0.015),
        normalError(0.01),
        incrementalNormalError(0.05),
        minimumOcclusionCount(10),
        occlusionOutlierCount(10)
    {}
};

} /* namespace consist */


#endif /* HYPOTHESISTESTEROPTIONS_H_ */

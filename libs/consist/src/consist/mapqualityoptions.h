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

#ifndef MAPQUALITYOPTIONS_H_
#define MAPQUALITYOPTIONS_H_

namespace consist {

/**
 * \brief Evaluation options for the MapQuality functor
 * \see MapQuality
 */
struct MapQualityOptions {
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
     * \brief Number of iterations to pass to g2o when optimizing the map.
     *
     * The default value is 50.
     */
    int iterations;

    /**
     * \brief How many threads to use for the computation.
     *
     * If 0 the number of threads is chosen automatically. The default value is 0.
     */
    int threads;

    /**
     * \brief Put robust kernel on odometry (sequential) edges too.
     *
     * \c false by default.
     */
    bool odometryKernel;

    /**
     * \brief Be verbose when computing the consistency.
     *
     * This will display timing and evaluation value statistics each time the
     * functor is called. This is enabled by default.
     */
    bool verbose;

    /**
     * \brief Initializes a new object with the default options.
     */
    MapQualityOptions() :
        maxRange(8),
        maxDistance(.2),
        nodeSpacing(1),
        iterations(50),
        threads(0),
        odometryKernel(false),
        verbose(true)
    {}
};

} /* namespace consist */

#endif /* MAPQUALITYOPTIONS_H_ */

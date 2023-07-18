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

#ifndef CONSIST_H_
#define CONSIST_H_

#include "foreach.h"
#include "support.h"
#include "visibility.h"
#include "parallelrunner.h"
#include "linesegment.h"
#include "probability.h"
#include "quadrilateral.h"
#include "inconsistencyse2.h"
#include "stopwatch.h"
#include "hypothesistesteroptions.h"
#include "hypothesistester.h"
#include "gridbisectionoptimizer.h"
#include "mapqualityoptions.h"
#include "mapquality.h"

/**
 * \brief Main namespace containing the Consist library functionalities
 */
namespace consist {}

/**
 * \mainpage Consist: a library for checking the consistency of SLAM maps
 *
 * \section intro Introduction
 *
 * This library has mainly two functionalities:
 * - The ability to determine which portions of a 2D SLAM map appear to be
 *   inconsistent.
 * - Estimating the optimal parameter to feed into robust mapping frameworks
 *   such as DCS or, more generally, M-estimators.
 *
 * \section hypotest Hypothesis testing
 *
 * Hypothesis testing on the map can be carried out on a g2o::SparseOptimizer
 * object. Only 2D objects will be considered, namely g2o::VertexSE2,
 * g2o::EdgeSE2 and g2o::RawLaser. Do note that for the hypothesis testing
 * to function at least a subset of the vertices need to have a corresponding
 * g2o::RawLaser in the user data, as that is the type of object from which
 * the data is sourced for consistency evaluation.
 *
 * If the SLAM map is kept in the variable \c map, than a sample usage of
 * this functionality can be achieved as follows:
 * \code
 * #include <consist.h>
 *
 * using namespace consist;
 * // [...]
 * HypothesisTester ht;
 * ht.setMap(map);
 * ht.setOptions(someOptions);    // Optional
 * ht.computeConsistencyMatrix();
 * std::vector<int> inconsistentIds = ht.binomialHypothesisTest();
 * \endcode
 *
 * Refer to consist::HypothesisTesterOptions for an exhaustive description
 * of the parameters that HypothesisTester can be configured with. By default
 * the options are initialized to a set of confidence values which have proven
 * to be particularly effective (in terms of precision-specificity) for
 * dealing with static datasets. Dynamic datasets generally require larger
 * confidences to cope with the increased amount of noise.
 *
 * \section dcsopt Robust parameter optimization
 *
 * As with the hypothesis testing, robust parameter optimization is carried
 * out on a g2o::SparseOptimizer with g2o::RawLaser data recordings. The
 * optimizer is mostly targeted to DCS, but can also be used for any single
 * parameter robust kernel.
 *
 * If the SLAM map is kept in the variable \c map, than a sample usage of
 * this functionality can be achieved as follows:
 * \code
 * #include <consist.h>
 *
 * using namespace consist;
 * // [...]
 * MapQuality mq(map);
 * mq.setOptions(someOptions);    // Optional
 *
 * GridBisectionOptimizer gbo;
 * gbo.setBounds(minPhi, maxPhi); // [minPhi, maxPhi] interval
 * gbo.setSteps(nSteps);          // Number of grid evaluations in the interval
 * gbo.setStepCriterion(crit)     // GridBisectionOptimizer::Linear or
 *                                // GridBisectionOptimizer::Quadratic
 * gbo.setAcceptanceError(err);   // Acceptance error for bisection convergence
 *
 * double optimalPhi = gbo.optimize(mq);
 * \endcode
 *
 * Refer to consist::MapQualityOptions for an exhaustive description
 * of the parameters that MapQuality can be configured with. Do note that
 * MapQuality is implemented as a functor, hence it can be used as the
 * objective function of any other derivative free optimization tool.
 *
 * \section tools Shipped tools
 *
 * Consist comes with two command line tools for testing the consistency
 * of maps (\c tester) and for optimizing the parameter of DCS
 * (\c optimizer).
 *
 * Respectively refer to <tt>./tester --help</tt> and to
 * <tt>./optimizer --help</tt> for usage instruction, Namely:
 * \verbatim
$ ./tester --help
Usage: ./tester [options] [gm2dl file]

Arguments:
  [gm2dl file]
          Dataset filename to load, in gm2dl format [required]
Options:
  -g, --gui
          Show the graphical user interface.
  -d SPACING, --spacing=SPACING
          Minimum node spacing between subsequent laser scans to be considered
          when computing the map quality. Effectively decimates the input scans,
          the spacing is considered in terms of order of appearence in the
          dataset. Note that 1 accepts all laser scans.
          * [default 1]
  -l LOCALITY, --locality=LOCALITY
          How many nodes to look forwards/backwards in the incremental only map
          for the distribution parameters estimation procedure.
          * [default 20]
  -o OPTIMIZE, --optimize=OPTIMIZE
          Optimize the full map before evaluating its consistency. Takes as
          argument the maximum number of iterations for g2o optimization. Note
          that the incremental map is always optimized, regardless of this
          option.
          * [default off]
  --dcs=DCS
          Enable DCS and use the argument of this option as the phi parameter.
          This option requires --optimize to have an effect.
          * [default off]
  -t THREADS, --threads=THREADS
          Number of threads to use for computation, 0 to autodetect
          * [default 0]
  --normal-err=NORMAL-ERR
          1 - confidence for a single normal hypothesis test
          * [default 0.010]
  --incr-normal-err=INCR-NORMAL-ERR
          1 - confidence for estimating the distribution parameters. This value
          should be larger than --normal-err to account for the fact that random
          samples from a normal distribution do not necessarily fall at the
          maximum allowed error. Choosing a smaller or equal value would result
          in an overly optimistic estimate
          * [default 0.050]
  --binomial-prob=BINOMIAL-PROB
          Probability of failure of the normal hypothesis test to feed into the
          binomial test. Ideally, this should be equivalent to --normal-err, in
          reality this value should be slightly larger than --normal-err in
          order to account for the error introduced by approximating a finite
          sum of random variables with a normal random variable
          * [default 0.015]
  --binomial-err=BINOMIAL-ERR
          1 - confidence for the binomial hypothesis test
          * [default 0.010]
  --g2o-verbose
          Let g2o be verbose.
  -v, --verbose
          Be verbose (does not entail g2o verbosity).
  -h, --help
          Show this help screen
\endverbatim
 * \verbatim
$ ./optimizer --help
Usage: ./optimizer [options] [gm2dl file]

Arguments:
  [gm2dl file]
          Dataset filename to load, in gm2dl format. [required]
Options:
  -m MIN, --min=MIN
          Minimum phi search value.
          * [default 1e-06]
  -M MAX, --max=MAX
          Maximum phi search value.
          * [default 16.00]
  -s STEPS, --steps=STEPS
          Number of phi steps to be considered for grid search.
          * [default 40]
  -o OPT-COND, --opt-cond=OPT-COND
          Optimality condition for bisection search. Terminates the optimization
          when the difference in phi between subsequent evaluations is smaller
          (in absolute value) than this parameter.
          * [default 0.0001]
  -l, --linear
          Use a linearly spaced grid, rather than a quadratically spaced one,
          when doing grid search.
  -d SPACING, --spacing=SPACING
          Minimum node spacing between subsequent laser scans to be considered
          when computing the map quality. Effectively decimates the input scans,
          the spacing is considered in terms of order of appearence in the
          dataset. Note that 1 accepts all laser scans.
          * [default 1]
  -i ITERATIONS, --iterations=ITERATIONS
          Maximum number of iterations to use for g2o optimization.
          * [default 50]
  -t THREADS, --threads=THREADS
          Number of threads to use for map quality evaluation, 0 to autodetect.
          * [default 0]
  --g2o-verbose
          Let g2o be verbose.
  -v, --verbose
          Be verbose (does not entail g2o verbosity).
  -h, --help
          Show this help screen.
\endverbatim
 */

#endif /* CONSIST_H_ */

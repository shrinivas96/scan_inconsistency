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

#include "optimizeroptions.h"
#include "shared/argumentparser.h"
#include "consist/support.h"
#include <fstream>
#include <iostream>

OptimizerOptions::OptimizerOptions(int argc, char **argv) :
    minphi(1e-6),
    maxphi(16),
    optcond(1e-4),
    steps(40),
    iterations(50),
    nodeSpacing(1),
    fileName(NULL),
    threads(0),
    linear(false),
    odometryKernel(false),
    verbose(false),
    g2oVerbose(false),
    abort(false),
    abortValue(0)
{
    ArgumentParser ap;
    bool help = false;

    ap.addOption("min",         'm', minphi,
                 "Minimum phi search value.\n* [default %g]");

    ap.addOption("max",         'M', maxphi,
                 "Maximum phi search value.\n* [default %.2f]");

    ap.addOption("steps",       's', steps,
                 "Number of phi steps to be considered for grid search.\n* [default %d]");

    ap.addOption("opt-cond",    'o', optcond,
                 "Optimality condition for bisection search. Terminates the optimization "
                 "when the difference in phi between subsequent evaluations is smaller "
                 "(in absolute value) than this parameter.\n* [default %g]");

    ap.addOption("linear",      'l', linear,
                 "Use a linearly spaced grid, rather than a quadratically spaced one, "
                 "when doing grid search.");

    ap.addOption("odokernel",   'k', odometryKernel,
                 "Put robust kernel on odometry (sequential) edges too.");

    ap.addOption("spacing",     'd', nodeSpacing,
                 "Minimum node spacing between subsequent laser scans to be considered "
                 "when computing the map quality. Effectively decimates the input scans, "
                 "the spacing is considered in terms of order of appearence in the dataset. "
                 "Note that 1 accepts all laser scans.\n* [default %d]");

    ap.addOption("iterations",  'i', iterations,
                 "Maximum number of iterations to use for g2o optimization.\n* [default %d]");

    ap.addOption("threads",     't', threads,
                 "Number of threads to use for map quality evaluation, 0 to autodetect."
                 "\n* [default %d]");

    ap.addOption("g2o-verbose",   0, g2oVerbose,
                 "Let g2o be verbose.");

    ap.addOption("verbose",     'v', verbose,
                 "Be verbose (does not entail g2o verbosity).");

    ap.addOption("help",        'h', help,
                 "Show this help screen.");

    ap.addArgument(fileName, "[gm2dl file]",
                   "Dataset filename to load, in gm2dl format. [required]");

    ap.parse(argc, argv);

    if(help) {
        ap.printHelp();
        abort = true;
        abortValue = 0;
    } else {
        checkOptions();
    }
}

void OptimizerOptions::checkOptions()
{
    abort = true;
    abortValue = -1;
    if(minphi > maxphi) {
        std::cerr << "Error: maximum phi needs to be larger than minimum sqrt phi" << std::endl;
    } else if(minphi < 0) {
        std::cerr << "Error: minimum phi needs to be greater or equal to zero" << std::endl;
    } else if(fileName == NULL) {
        std::cerr << "Error: must specify a file name" << std::endl;
    } else if(!ArgumentParser::fileExists(fileName)) {
        std::cerr << "Error: file '" << fileName << "' does not exist" << std::endl;
    } else {
        abort = false;
        abortValue = 0;
    }
}


consist::MapQualityOptions OptimizerOptions::mapQualityOptions() const
{
    consist::MapQualityOptions opts;
    opts.iterations = iterations;
    // opts.maxDistance = ...
    // opts.maxRange = ...
    opts.nodeSpacing = nodeSpacing;
    opts.threads = threads;
    opts.odometryKernel = odometryKernel;
    opts.verbose = verbose;
    return opts;
}

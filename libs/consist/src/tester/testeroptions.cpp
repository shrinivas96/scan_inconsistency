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

#include "testeroptions.h"
#include "shared/argumentparser.h"
#include <fstream>
#include <iostream>
#include <cmath>

TesterOptions::TesterOptions(int argc, char **argv) :
#ifdef GUI_SUPPORT
    gui(false),
#endif /* GUI_SUPPORT */
    optimize(0),
    dcs(-INFINITY),
    fileName(NULL),
    g2oVerbose(false),
    abort(false),
    abortValue(0)
{

    ArgumentParser ap;
    bool help = false;

#ifdef GUI_SUPPORT
    ap.addOption("gui",             'g', gui,
                 "Show the graphical user interface.");
#endif /* GUI_SUPPORT */

    ap.addOption("spacing",         'd', hypothesisOptions.nodeSpacing,
                 "Minimum node spacing between subsequent laser scans to be considered "
                 "when computing the map quality. Effectively decimates the input scans, "
                 "the spacing is considered in terms of order of appearence in the dataset. "
                 "Note that 1 accepts all laser scans.\n* [default %d]");

    ap.addOption("locality",        'l', hypothesisOptions.locality,
                 "How many nodes to look forwards/backwards in the incremental only map for the "
                 "distribution parameters estimation procedure.\n* [default %d]");

    ap.addOption("optimize",        'o', optimize,
                 "Optimize the full map before evaluating its consistency. Takes as argument "
                 "the maximum number of iterations for g2o optimization. Note that the "
                 "incremental map is always optimized, regardless of this option."
                 "\n* [default off]");

    ap.addOption("dcs",             0, dcs,
                 "Enable DCS and use the argument of this option as the phi parameter. "
                 "This option requires --optimize to have an effect.\n* [default off]");

    ap.addOption("threads",         't', hypothesisOptions.threads,
                 "Number of threads to use for computation, 0 to autodetect\n* [default %d]");

    ap.addOption("normal-err",       0,  hypothesisOptions.normalError,
                 "1 - confidence for a single normal hypothesis test\n* [default %.3f]");

    ap.addOption("incr-normal-err",  0,  hypothesisOptions.incrementalNormalError,
                 "1 - confidence for estimating the distribution parameters. This value should be "
                 "larger than --normal-err to account for the fact that random samples from a "
                 "normal distribution do not necessarily fall at the maximum allowed error. "
                 "Choosing a smaller or equal value would result in an overly optimistic estimate"
                 "\n* [default %.3f]");

    ap.addOption("binomial-prob",    0,  hypothesisOptions.binomialProbability,
                 "Probability of failure of the normal hypothesis test to feed into the binomial "
                 "test. Ideally, this should be equivalent to --normal-err, in reality this "
                 "value should be slightly larger than --normal-err in order to account for the "
                 "error introduced by approximating a finite sum of random variables with a "
                 "normal random variable\n* [default %.3f]");

    ap.addOption("binomial-err",     0,  hypothesisOptions.binomialError,
                 "1 - confidence for the binomial hypothesis test\n* [default %.3f]");

    ap.addOption("g2o-verbose",   0, g2oVerbose,
                 "Let g2o be verbose.");

    ap.addOption("verbose",     'v', hypothesisOptions.verbose,
                 "Be verbose (does not entail g2o verbosity).");

    ap.addOption("help",            'h', help,
                 "Show this help screen");

    ap.addArgument(fileName, "[gm2dl file]",
                   "Dataset filename to load, in gm2dl format [required]");

    ap.parse(argc, argv);

    if(help) {
        ap.printHelp();
        abort = true;
        abortValue = 0;
    } else {
        checkOptions();
    }
}

void TesterOptions::checkOptions()
{
    abort = true;
    abortValue = -1;
    if(hypothesisOptions.normalError <= 0 ||
            hypothesisOptions.normalError >= 1) {
        std::cerr << "Error: --normal-err needs to greater than 0 and smaller than 1" << std::endl;
    } else if(hypothesisOptions.incrementalNormalError <= 0 ||
            hypothesisOptions.incrementalNormalError >= 1) {
        std::cerr << "Error: --incr-normal-err needs to greater than 0 and smaller than 1" << std::endl;
    } else if(hypothesisOptions.binomialProbability <= 0 ||
            hypothesisOptions.binomialProbability >= 1) {
        std::cerr << "Error: --binomial-prob needs to greater than 0 and smaller than 1" << std::endl;
    } else if(hypothesisOptions.binomialError <= 0 ||
            hypothesisOptions.binomialError >= 1) {
        std::cerr << "Error: --binomial-err needs to greater than 0 and smaller than 1" << std::endl;
    } else if(fileName == NULL) {
        std::cerr << "Error: must specify a file name" << std::endl;
    } else if(!ArgumentParser::fileExists(fileName)) {
        std::cerr << "Error: file '" << fileName << "' does not exist" << std::endl;
    } else {
        abort = false;
        abortValue = 0;

        if(hypothesisOptions.incrementalNormalError < hypothesisOptions.normalError) {
            std::cerr << "Warning: --incr-normal-err should be larger than --normal-err" << std::endl;
        } else if(hypothesisOptions.binomialProbability < hypothesisOptions.normalError) {
            std::cerr << "Warning: --binomial-prob should be larger than --normal-err" << std::endl;
        }
    }
}

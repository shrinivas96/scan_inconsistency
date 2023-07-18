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

#ifndef OPTIMIZEROPTIONS_H_
#define OPTIMIZEROPTIONS_H_

#include "consist/mapqualityoptions.h"

struct OptimizerOptions {
    double minphi;
    double maxphi;
    double optcond;
    int steps;
    int iterations;
    int nodeSpacing;
    char *fileName;
    int threads;
    bool linear;
    bool odometryKernel;
    bool verbose;
    bool g2oVerbose;
    bool abort;
    int abortValue;


    OptimizerOptions(int argc, char **argv);
    void checkOptions();
    consist::MapQualityOptions mapQualityOptions() const;
};


#endif /* OPTIMIZEROPTIONS_H_ */

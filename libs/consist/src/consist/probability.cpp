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

#include "probability.h"
#include <math.h>

namespace consist {

namespace probability {

static inline double rationalApproxAbramowitzStegun(double t) {
    static const double c[] = {2.515517, 0.802853, 0.010328};
    static const double d[] = {1.432788, 0.189269, 0.001308};
    return t - (c[0] + t * (c[1] + t * c[2])) /
            (1 + t * (d[0] + t * (d[1] + t * d[2])));
}

double inverseNormalCDF(double p) {
    if(p < 0.5) {
        return -rationalApproxAbramowitzStegun(sqrt(-2 * log(p)));
    } else {
        return rationalApproxAbramowitzStegun(sqrt(-2 * log(1 - p)));
    }
}

double halfNormalMean(double s2) {
    return sqrt(2 * s2 / M_PI);
}

double halfNormalVariance(double s2) {
    return s2 * (1 - 2 / M_PI);
}

double binomialPMF(double p, int n, int k) {
    /* lgamma requires c99, hopefully you have it */
    return exp(lgamma(n + 1) - lgamma(k + 1) - lgamma(n - k + 1) + k * log(p) + (n - k) * log(1 - p));
}

int binomialCutoff(double p, int n, double pvalue) {
    int cutoff = n + 1;
    double thisp = 0, prevp = 0;
    while(thisp + prevp < pvalue) {
        thisp += prevp;
        prevp = binomialPMF(p, n, --cutoff);
    }
    return cutoff + 1;
}

} /* namespace probability */

BinomialCutoffCacher::BinomialCutoffCacher(int N, double p, double pvalue) :
    _N(N), _p(p), _pvalue(pvalue), _cutoffs(N, NAN)
{
    _cutoffs[0] = 0;
}

BinomialCutoffCacher::~BinomialCutoffCacher()
{
}

int BinomialCutoffCacher::cutoff(int n)
{
    if(n < _N) {
        if(_cutoffs[n] == _cutoffs[n]) { /* !std::isnan(_cutoffs[n]) */
            return _cutoffs[n];
        } else {
            return _cutoffs[n] = probability::binomialCutoff(_p, n, _pvalue);
        }
    } else {
        return probability::binomialCutoff(_p, n, _pvalue);
    }
}

} /* namespace consist */

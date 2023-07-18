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

#include <vector>
#include <math.h>

namespace consist {

template <typename Function>
double GridBisectionOptimizer::optimize(Function func)
{
    const double scaledmin = scale(_min), scaledmax = scale(_max);
    const double step = std::max(1e-10, (scaledmax - scaledmin) / _steps);

    std::vector<double> xvalues;
    std::vector<double> yvalues;
    for(double x = scaledmin; x <= scaledmax; x += step) {
        double y = func(invscale(x));
        xvalues.push_back(x);
        yvalues.push_back(y);
    }

    int minidx = -1;
    double minval = INFINITY;
    for(std::size_t i = 0; i < yvalues.size(); i++) {
        if(yvalues[i] < minval) {
            minval = yvalues[i];
            minidx = i;
        }
    }

    double xstart  = xvalues[std::max(0, minidx - 1)];
    double xend    = xvalues[std::min((int) xvalues.size() - 1, minidx + 1)];
    double xmiddle = xvalues[minidx];
    double ystart  = yvalues[std::max(0, minidx - 1)];
    double yend    = yvalues[std::min((int) xvalues.size() - 1, minidx + 1)];
    double ymiddle = yvalues[minidx];

    while(invscale(xend) - invscale(xstart) > _accept) {
        double x1 = (xstart + xmiddle) / 2;
        double x2 = (xmiddle + xend) / 2;
        double y1 = func(invscale(x1));
        double y2 = func(invscale(x2));

        if(y1 < y2) {
            xend    = xmiddle;
            yend    = ymiddle;
            xmiddle = x1;
            ymiddle = y1;
        } else {
            xstart  = xmiddle;
            ystart  = ymiddle;
            xmiddle = x2;
            ymiddle = y2;
        }
    }

    if(ystart < ymiddle && ystart < yend) {
        return invscale(xstart);
    } else if(ymiddle < ystart && ymiddle < yend) {
        return invscale(xmiddle);
    } else {
        return invscale(xend);
    }
}

} /* namespace consist */

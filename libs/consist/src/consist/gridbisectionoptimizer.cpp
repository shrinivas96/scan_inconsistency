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

#include "gridbisectionoptimizer.h"
#include <cmath>

namespace consist {

double GridBisectionOptimizer::scale(double x)
{
    if(_criterion == Quadratic) {
        return std::sqrt(x);
    } else /* if(_criterion == Linear) */ {
        return x;
    }
}

double GridBisectionOptimizer::invscale(double x)
{
    if(_criterion == Quadratic) {
        return x * x;
    } else /* if(_criterion == Linear) */ {
        return x;
    }
}

} /* namespace consist */

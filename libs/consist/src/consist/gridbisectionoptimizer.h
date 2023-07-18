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

#ifndef GRIDBISECTIONOPTIMIZER_H_
#define GRIDBISECTIONOPTIMIZER_H_

namespace consist {

/**
 * \brief Grid and bisection function optimizer
 *
 * An instance of this class allows to empirically determine the minimum
 * of a function by evaluating it at regular grid points (either linearly or
 * quadratically spaced) on a bounded interval. Once the function is
 * evaluated at the selected points, a bisection procedure is carried out
 * around the minimum of the grid search to refine the value.
 * This implementation handles only single variable scalar functions
 * \f$f:\mathbb{R}\to\mathbb{R}\f$. The independent variable is referred to as
 * \f$x\f$.
 */
class GridBisectionOptimizer {
public:
    /** \brief Point spacing criterion for grid search. */
    enum StepCriterion {
        /** \brief Linearly spaced grid. */
        Linear,
        /** \brief Quadratically spaced grid. */
        Quadratic
    };

    /**
     * \brief Construct a GridBisectionOptimizer object.
     *
     * By default the grid is set to 10 quadratically spaced points in the [0,1]
     * interval range. Optimization terminates when the \f$x\f$ tolerance is
     * smaller than 1e-4.
     */
    GridBisectionOptimizer() :
        _min(0), _max(1), _steps(10), _accept(1e-4), _criterion(Quadratic) {}

    virtual ~GridBisectionOptimizer() {}

    /**
     * \brief Set the bounds of the interval over which to evaluate the function.
     *
     * \param min Minimum \f$x\f$ value.
     * \param max Maximum \f$x\f$ value.
     */
    void setBounds(double min, double max) { _min = min; _max = max; }

    /**
     * \brief Set the discretization of the grid.
     *
     * \param steps The number of points to evaluate in the bounded interval.
     */
    void setSteps(int steps) { _steps = steps; }

    /**
     * \brief Set the termination condition.
     *
     * Bisection stops when subsequent \f$x\f$ values differ by less than \p accept.
     *
     * \param accept Acceptance error threshold.
     */
    void setAcceptanceError(double accept) { _accept = accept; }

    /**
     * \brief Set the grid spacing criterion (linear or quadratic).
     *
     * \param criterion Spacing criterion to use.
     */
    void setStepCriterion(StepCriterion criterion) { _criterion = criterion; }

    /**
     * \brief Optimize a function.
     *
     * Optimize the function \p fun by executing a grid search followed by a
     * bisection search. The search will be executed according to the parameters
     * defined for this GridBisectionOptimizer object.
     *
     * \param fun This is either a functor or a function. Either way it needs
     *            to take a \c double argument \f$x\f$ and needs to return a
     *            \c double value \f$f(x)\f$.
     * \return The optimal \f$x\f$ value.
     */
    template <typename Function>
    double optimize(Function fun);

private:
    double scale(double x);
    double invscale(double x);

private:
    double _min, _max;
    int _steps;
    double _accept;
    StepCriterion _criterion;
};

} /* namespace consist */

#include "gridbisectionoptimizer.hpp"

#endif /* GRIDBISECTIONOPTIMIZER_H_ */

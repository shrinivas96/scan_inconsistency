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

#ifndef PROBABILITY_H_
#define PROBABILITY_H_

#include <vector>

namespace consist {

/**
 * \brief Functions which deal with probabilistic inference
 */
namespace probability {

/**
 * \brief Inverse CDF of a standardized normal distribution.
 *
 * The inverse cumulative distribution function \f$\Phi^{-1}(p)\f$ is
 * implemented according to the Abramowitz and Stegun rational approximation.
 * The error is guaranteed to be smaller than 4.5e-4 for any \f$p\in(0,1)\f$.
 *
 * \param p Probability value in the open interval (0,1)
 * \return The value of \f$\Phi^{-1}(p)\f$, with an absolute error smaller
 *         than 4.5e-4.
 */
double inverseNormalCDF(double p);

/**
 * \brief Computes the mean of a half normal distribution.
 *
 * \param s2 The (quadratic) scale parameter of the half normal distribution.
 * \return \f$\sqrt{2\frac{s^2}{\pi}}\f$
 */
double halfNormalMean(double s2);

/**
 * \brief Computes the variance of a half normal distribution.
 *
 * \param s2 The (quadratic) scale parameter of the half normal distribution.
 * \return \f$s^2\left(1-\frac{2}{\pi}\right)\f$
 */
double halfNormalVariance(double s2);

/**
 * \brief Computes the binomial PMF.
 *
 * Computes the value of the probability mass function of a binomial
 * distribution with given p, n and k, i.e. it computes the following value:
 * \f[
 *      {n \choose k} p^k (1-p)^{n-k}
 * \f]
 * The computation is done by using logarithms in order to maintain numerical
 * stability.
 *
 * \param p Success probability of a binomial trial.
 * \param n Number of trials.
 * \param k Number of successful trials.
 * \return Binomial PMF.
 */
double binomialPMF(double p, int n, int k);

/**
 * \brief Computes the maximum number of accepted successful trials for a
 *        given p-value.
 *
 * Technically, this function computes the inverse cumulative distribution
 * function of a binomial evaluated at 1 - \p pvalue. This particular choice
 * of having 1 - \p pvalue rather that \p pvalue is to keep full IEEE
 * floating point precision even when the argument of the inverse CDF is
 * close to 1 (which is what we're interested in).
 * If we denote \p pvalue by \f$\alpha\f$ then this function computes the
 * following:
 * \f[
 *      \min_{0\le m\le n} \left\{m\left|\,\sum_{k=m+1}^n
 *          {n \choose k} p^k (1-p)^{n-k}\le \alpha\right.\right\}
 * \f]
 *
 * \param p Success probability of a binomial trial.
 * \param n Number of trials.
 * \param pvalue \f$\alpha\f$ in the above formula.
 * \return The value of the above formula.
 */
int binomialCutoff(double p, int n, double pvalue);

} /* namespace probability */

/**
 * \brief Cacher object for computing binomial cutoff values.
 *
 * Caches the result of probability::binomialCutoff() for relatively small
 * numbers of trials, in order to avoid recomputing them all the time.
 *
 * \see probability::binomialCutoff()
 */
class BinomialCutoffCacher {
public:
    /**
     * \brief Initialize BinomialCutoffCacher
     *
     * The object is initialized so that any value of \p n that would have
     * been passed to probability::binomialCutoff() will be cached if
     * \p n < \p N.
     *
     * \param N Maximum \p n to be cached.
     * \param p Success probability of a binomial trial.
     * \param pvalue The \p pvalue parameter to be passed to
     *               probability::binomialCutoff().
     */
    BinomialCutoffCacher(int N, double p, double pvalue);
    virtual ~BinomialCutoffCacher();

    /**
     * \brief Compute the binomial cutoff for a given \p n
     *
     * If the cutoff value has been cached for the particular \p n, this
     * method will return the cached value. Otherwise this method will
     * call return probability::binomialCutoff(p, n, pvalue).
     *
     * \param n Number of trials.
     * \return The binomial cutoff value.
     */
    int cutoff(int n);

private:
    int _N;
    std::vector<double> _cutoffs;
    double _p, _pvalue;
};

} /* namespace consist */

#endif /* PROBABILITY_H_ */

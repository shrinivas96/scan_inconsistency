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

#include "hypothesistester.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/data/robot_laser.h>
#include "visibility.h"
#include "foreach.h"
#include "probability.h"
#include "parallelrunner.h"

namespace consist {

HypothesisTester::HypothesisTester() :
        _full(NULL), _incremental(NULL)
{
}

HypothesisTester::~HypothesisTester()
{
    delete _full;
    delete _incremental;
}

void HypothesisTester::setOptions(const HypothesisTesterOptions &opts)
{
    _opts = opts;
}

const HypothesisTesterOptions &HypothesisTester::options() const
{
    return _opts;
}

void HypothesisTester::setMap(const g2o::SparseOptimizer *so)
{
    _full = clone(so, false);
    _incremental = clone(so, true);
    _visibilitiesFull = addVisibilities(_full);
    _visibilitiesIncremental = addVisibilities(_incremental);
}


g2o::SparseOptimizer *HypothesisTester::fullMap()
{
    return _full;
}

g2o::SparseOptimizer *HypothesisTester::incrementalMap()
{
    return _incremental;
}

const Eigen::MatrixXd &HypothesisTester::consistencyMatrix() const
{
    return _consistency;
}

g2o::SparseOptimizer *HypothesisTester::clone(
        const g2o::SparseOptimizer *so, bool incrementalOnly)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    typedef g2o::OptimizationAlgorithmGaussNewton OptimizationAlgorithm;

    g2o::SparseOptimizer *soclone = new g2o::SparseOptimizer();
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithm *solverAlg = new OptimizationAlgorithm(blockSolver);

    linearSolver->setBlockOrdering(false);
    solverAlg->setWriteDebug(false);
    soclone->setAlgorithm(solverAlg);
    soclone->setVerbose(false);

    /* Assumption: node id reflects the order of appearence */
    std::map<int, const g2o::HyperGraph::Vertex *> orderedVertices(
            so->vertices().begin(), so->vertices().end());
    double ticker = _opts.nodeSpacing;

    fforeach_map(int id, const g2o::HyperGraph::Vertex *vert, orderedVertices) {
        const g2o::VertexSE2 *v = dynamic_cast<const g2o::VertexSE2 *>(vert);
        if(v) {
            g2o::VertexSE2  *nv   = new g2o::VertexSE2;
            const g2o::RobotLaser *scan = support::findFirstDatum<g2o::RobotLaser>(v);
            nv->setEstimate(v->estimate());
            nv->setId(v->id());
            if(scan) {
                if(ticker >= _opts.nodeSpacing) {
                    g2o::RobotLaser *nscan = new g2o::RobotLaser(*scan);
                    nv->setUserData(nscan);
                    nscan->setNext(NULL);
                    ticker -= _opts.nodeSpacing;
                }
                ticker++;
            }
            soclone->addVertex(nv);
        }
    }

    if(orderedVertices.size() > 0) {
        soclone->vertex(orderedVertices.begin()->first)->setFixed(true);
    }

    fforeach(const g2o::HyperGraph::Edge *edge, so->edges()) {
        const g2o::EdgeSE2 *e = dynamic_cast<const g2o::EdgeSE2 *>(edge);

        /* Assumption: subsequent nodes need to have an id differing by at most 1 */
        if(e && (!incrementalOnly || std::abs(e->vertex(0)->id() - e->vertex(1)->id()) <= 1)) {
            g2o::EdgeSE2 *ne = new g2o::EdgeSE2(*e);
            ne->setRobustKernel(NULL);
            ne->setVertex(0, soclone->vertex(ne->vertex(0)->id()));
            ne->setVertex(1, soclone->vertex(ne->vertex(1)->id()));
            soclone->addEdge(ne);
        }
    }

    if(incrementalOnly) {
        g2o::EstimatePropagatorCostOdometry propagator(soclone);
        soclone->computeInitialGuess(propagator);
    }

    return soclone;
}

std::vector<Visibility *> HypothesisTester::addVisibilities(g2o::SparseOptimizer *so)
{
    std::vector<Visibility *> visibilities;

    fforeach_map(int id, g2o::HyperGraph::Vertex *vert, so->vertices()) {
        g2o::VertexSE2 *v = dynamic_cast<g2o::VertexSE2 *>(vert);
        g2o::RawLaser *scan = support::findFirstDatum<g2o::RawLaser>(vert);
        if(v && scan) {
            Visibility *vis = new Visibility(
                    scan, v->estimate(), _opts.maxRange, _opts.maxDistance);
            g2o::OptimizableGraph::Data *datum = scan;
            while(datum->next()) {
                datum = datum->next();
            }
            datum->setNext(vis);
            visibilities.push_back(vis);
        }
    }
    return visibilities;
}

template <int STAGE>
void HypothesisTester::progressDisplayer(int index, int max, HypothesisTester *) {
    std::cerr << index + 1 << " / " << max << " (" << STAGE << ")\r" << std::flush;
}

void HypothesisTester::incrementalParametersThread(int index, HypothesisTester *ht) {
    const double invNormal = -probability::inverseNormalCDF(ht->_opts.incrementalNormalError);

    int start = std::max(0, index - ht->_opts.locality);
    int end   = std::min(index + ht->_opts.locality, (int) ht->_visibilitiesIncremental.size());
    const Visibility &scanvis = *ht->_visibilitiesIncremental[index];

    double sampleMean = probability::halfNormalMean(0.03);          // Prior guess
    double sampleVariance = probability::halfNormalVariance(0.03);  // Prior guess
    std::vector<double> sampleMeans;
    std::vector<int> sampleCounts;

    for(int i = start; i < end; i++) {
        const Visibility &other = *ht->_visibilitiesIncremental[i];
        if(i != index && scanvis.maybeOverlapsWith(other)) {
            std::vector<double> occlusions1 = other.polyOcclusions(scanvis);
            std::vector<double> occlusions2 = scanvis.polyOcclusions(other);
            occlusions1.insert(occlusions1.end(), occlusions2.begin(), occlusions2.end());
            if(occlusions1.size() >= ht->_opts.minimumOcclusionCount) {
                double m = support::mean(occlusions1);
                double s = support::variance(occlusions1);
                sampleVariance = std::max(sampleVariance, s);
                sampleMeans.push_back(m);
                sampleCounts.push_back(occlusions1.size());
            }
        }
    }

    for(size_t i = 0; i < sampleMeans.size(); i++) {
        sampleMean = std::max(sampleMean,
            sampleMeans[i] - std::sqrt(sampleVariance / sampleCounts[i]) * invNormal);
    }

    ht->_means[index]     = sampleMean;
    ht->_variances[index] = sampleVariance;
}

void HypothesisTester::consistencyMatrixThread(int index, HypothesisTester *ht) {
    const Visibility &scanvis = *ht->_visibilitiesFull[index];

    for(size_t i = index + 1; i < ht->_visibilitiesFull.size(); i++) {
        const Visibility &other = *ht->_visibilitiesFull[i];
        if(scanvis.maybeOverlapsWith(other)) {
            double maxmean = ht->_means[index], maxvariance = ht->_variances[index];
            if(ht->_means[i] + 3 * std::sqrt(ht->_variances[i]) > maxmean + 3 * std::sqrt(maxvariance)) {
                maxmean = ht->_means[i];
                maxvariance = ht->_variances[i];
            }
            std::vector<double> occlusions = other.polyOcclusions(scanvis);
            std::vector<double> second = scanvis.polyOcclusions(other);
            occlusions.insert(occlusions.end(), second.begin(), second.end());

            if(occlusions.size() < ht->_opts.minimumOcclusionCount) {
                ht->_consistency(index, i) = -INFINITY;
                ht->_consistency(i, index) = -INFINITY;
            } else {
                std::sort(occlusions.begin(), occlusions.end());
                occlusions.resize(occlusions.size() - ht->_opts.occlusionOutlierCount);
                double thisMean = support::mean(occlusions);
                double standardizedMij = (thisMean - maxmean) / std::sqrt(maxvariance / occlusions.size());
                ht->_consistency(index, i) = standardizedMij;
                ht->_consistency(i, index) = standardizedMij;
            }
        }
    }
}

void HypothesisTester::computeConsistencyMatrix()
{
    _consistency = Eigen::MatrixXd::Constant(
            _visibilitiesFull.size(), _visibilitiesFull.size(), -INFINITY);
    _means      = Eigen::VectorXd::Zero(_visibilitiesFull.size());
    _variances  = Eigen::VectorXd::Zero(_visibilitiesFull.size());

    ParallelRunner<HypothesisTester> pr1(incrementalParametersThread);
    ParallelRunner<HypothesisTester> pr2(consistencyMatrixThread);
    pr1.setJobCount(_visibilitiesIncremental.size());
    pr2.setJobCount(_visibilitiesFull.size());
    pr1.setProgressFunction(progressDisplayer<1>);
    pr2.setProgressFunction(progressDisplayer<2>);

    std::cerr << support::cursorHide;
    pr1.run(this, _opts.threads);
    std::cerr << std::endl;
    pr2.run(this, _opts.threads);
    std::cerr << std::endl << support::cursorShow;
}


std::vector<int> HypothesisTester::binomialHypothesisTest(bool simpleFiltering)
{
    double normalThreshold = -probability::inverseNormalCDF(_opts.normalError);
    BinomialCutoffCacher cacher(100, _opts.binomialProbability, _opts.binomialError);


    std::vector<bool> tests;
    for(int i = 0; i < _consistency.rows(); i++) {
        int count = 0, failCount = 0;
        for(int j = 0; j < _consistency.cols(); j++) {
            if(_consistency(i, j) > -INFINITY) {
                count++;
                if(_consistency(i, j) > normalThreshold) {
                    failCount++;
                }
            }
        }

        if(count > 0 && failCount >= cacher.cutoff(count)) {
            tests.push_back(true);
        } else {
            tests.push_back(false);
        }
    }

    if(simpleFiltering) {
        const int locality = 4;
        for(size_t i = 0; i < tests.size(); i++) {
            if(tests[i]) {
                bool keep = false;
                for(int j = std::max(0, (int) i - locality);
                    j < std::min((int) i + locality, (int) tests.size() - 1); j++) {
                    keep |= (i != j) && tests[j];
                }
                tests[i] = keep;
            }
        }
    }

    std::vector<int> badids;

    std::map<int, const g2o::HyperGraph::Vertex *> orderedVertices(
            _full->vertices().begin(), _full->vertices().end());

    int i = 0;
    fforeach_map(int id, const g2o::HyperGraph::Vertex *v, orderedVertices) {
        if(support::findFirstDatum<Visibility>(v) && tests[i++]) {
            badids.push_back(id);
        }
    }


    return badids;
}



} /* namespace consist */

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

#include "mapquality.h"
#include <vector>
#include <map>
#include <Eigen/Core>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/types/data/raw_laser.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <stdio.h>
#include "foreach.h"
#include "parallelrunner.h"
#include "support.h"
#include "stopwatch.h"

namespace consist {

MapQuality::MapQuality() : _so(NULL), _iteration(0)
{
}

MapQuality::MapQuality(g2o::SparseOptimizer *so) : _so(so), _iteration(0)
{
}

MapQuality::~MapQuality()
{
}

void MapQuality::setMap(g2o::SparseOptimizer *so)
{
    _so = so;
}

void MapQuality::setOptions(const MapQualityOptions &opts)
{
    _opts = opts;
}

const MapQualityOptions &MapQuality::options() const
{
    return _opts;
}

int MapQuality::iterationCount() const
{
    return _iteration;
}

void MapQuality::resetIterationCount()
{
    _iteration = 0;
    _timing = 0;
}

/*
 * Wrapper class to keep track of the last optimization result.
 * Used to detect when optimization has failed.
 */
class OptimizationAlgorithmWithMemory : public g2o::OptimizationAlgorithm
{
public:
    explicit OptimizationAlgorithmWithMemory(g2o::OptimizationAlgorithm* alg) :
            g2o::OptimizationAlgorithm(), _alg(alg), _lastResult(g2o::OptimizationAlgorithm::OK) {}
    virtual ~OptimizationAlgorithmWithMemory() {}

    bool init(bool online = false) {
        return _alg->init(online);
    }

    g2o::OptimizationAlgorithm::SolverResult solve(int iteration, bool online = false) {
        _lastResult = _alg->solve(iteration, online);
        return _lastResult;
    }

    bool computeMarginals(
            g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv,
            const std::vector<std::pair<int, int> >& blockIndices) {
        return _alg->computeMarginals(spinv, blockIndices);
    }

    bool updateStructure(
            const std::vector<g2o::HyperGraph::Vertex*>& vset,
            const g2o::HyperGraph::EdgeSet& edges) {
        return _alg->updateStructure(vset, edges);
    }

    void printVerbose(std::ostream& os) const {
        _alg->printVerbose(os);
    }

    g2o::OptimizationAlgorithm *solver() {
        return _alg;
    }

    void setOptimizer() {
        _alg->setOptimizer(optimizer());
    }

    g2o::OptimizationAlgorithm::SolverResult lastResult() const {
        return _lastResult;
    }

private:
    g2o::OptimizationAlgorithm *_alg;
    g2o::OptimizationAlgorithm::SolverResult _lastResult;
};

void MapQuality::rowThread(int index, MapQuality *q)
{
    std::map<int, double> entries;
    const Visibility &scanvis = q->_visibilities[index];

    for(size_t i = index + 1; i < q->_visibilities.size(); i++) {
        const Visibility &other = q->_visibilities[i];
        if(scanvis.maybeOverlapsWith(other)) {
            std::vector<double> forward = other.polyOcclusions(scanvis);
            std::vector<double> backward = scanvis.polyOcclusions(other);

            entries[i] = support::sum(forward) + support::sum(backward);
        }
    }

    fforeach_map(int idx, double val, entries) {
        q->_pairwise(index, idx) = val;
        q->_pairwise(idx, index) = val;
    }
}

double MapQuality::computeMeasure()
{
    _pairwise = Eigen::MatrixXd::Zero(_visibilities.size(), _visibilities.size());
    ParallelRunner<MapQuality> pr(rowThread);
    pr.setJobCount(_visibilities.size());

    if(_opts.verbose) std::cerr << support::cursorHide;
    else pr.setProgressFunction(NULL);

    pr.run(this, _opts.threads);

    if(_opts.verbose) std::cerr << support::cursorShow;

    double msum = 0;
    int overlapCount = 0;
    for(int i = 0; i < _pairwise.rows(); i++) {
        for(int j = 0; j < _pairwise.cols(); j++) {
            if(_pairwise(i, j) > 0) {
                msum += _pairwise(i, j);
                overlapCount++;
            }
        }
    }
    if(_opts.verbose) std::cerr << std::endl;
    return msum / overlapCount;
}

double MapQuality::operator()()
{
    if(!_so)
        return INFINITY;

    /* If optimization has failed (and we can detect it) just return infinity */
    OptimizationAlgorithmWithMemory *wrapper =
            dynamic_cast<OptimizationAlgorithmWithMemory *>(_so->solver());
    double chi2 = _so->chi2();
    if((wrapper && wrapper->lastResult() == g2o::OptimizationAlgorithm::Fail) ||
            chi2 != chi2 /* c++11 std::isnan(chi2) */) {
        return INFINITY;
    }

    /* Determine the ids of all scans to keep */
    std::vector<int> keep;
    std::map<int, g2o::HyperGraph::Vertex *> orderedVertices;
    double ticker = _opts.nodeSpacing;
    
    /* Assumption: node id reflects the order of appearence */
    fforeach_map(int id, g2o::HyperGraph::Vertex *vert, _so->vertices()) {
        orderedVertices[id] = vert;
    }

    fforeach_map(int id, g2o::HyperGraph::Vertex *vert, orderedVertices) {
        g2o::VertexSE2 *v = dynamic_cast<g2o::VertexSE2 *>(vert);
        if(v && support::findFirstDatum<g2o::RawLaser>(v)) {
            if(ticker >= _opts.nodeSpacing) {
                keep.push_back(id);
                ticker -= _opts.nodeSpacing;
            }
            ticker++;
        }
    }

    _visibilities.clear();

    fforeach(int id, keep) {
        g2o::VertexSE2 *v = static_cast<g2o::VertexSE2 *>(_so->vertex(id));
        const g2o::RawLaser *scan = support::findFirstDatum<g2o::RawLaser>(v);
        _visibilities.push_back(Visibility(scan, v->estimate(), _opts.maxRange, _opts.maxDistance));
    }

    return computeMeasure();
}

double MapQuality::operator()(double phi)
{
    if(!_so)
       return INFINITY;

    std::map<g2o::OptimizableGraph::Edge *, double> previousKernelParams;

    /* Save current linearization point */
    g2o::OptimizableGraph::VertexContainer allVertices;
    fforeach_map(int id, g2o::HyperGraph::Vertex *v, _so->vertices()) {
        allVertices.push_back(static_cast<g2o::OptimizableGraph::Vertex *>(v));
    }
    _so->push(allVertices);

    /* Substitute optimization algorithm with wrapper */
    OptimizationAlgorithmWithMemory wrapper(_so->solver());
    _so->setAlgorithm(&wrapper);
    wrapper.setOptimizer();

    /* Set the kernel parameter for every EdgeSE2. If the edge has no kernel add a DCS kernel.
     * If the edges are pre-initialized with another kernel, say Cauchy, this will change the
     * kernel parameter without actually changing the kernel to DCS. */
    fforeach(g2o::HyperGraph::Edge *edge, _so->edges()) {
        g2o::EdgeSE2 *e = dynamic_cast<g2o::EdgeSE2 *>(edge);
        if(e) {
            if(!e->robustKernel()) {
                /* Assumption: odometry edges are edges which connect vertices differing in
                 * id by one. */
                bool odometryEdge = std::abs(e->vertex(0)->id() - e->vertex(1)->id()) == 1;

                if(!odometryEdge || (_opts.odometryKernel && odometryEdge)) {
                    previousKernelParams[e] = -INFINITY;
                    e->setRobustKernel(new g2o::RobustKernelDCS);
                    e->robustKernel()->setDelta(phi);
                }
            /* Always change kernel parameter on odometry edge if it already has a robust
             * kernel function. */
            } else {
                previousKernelParams[e] = e->robustKernel()->delta();
                e->robustKernel()->setDelta(phi);
            }
        }
    }

    Stopwatch timing;

    timing.start();
    /* Optimize with the new kernel parameter */
    _so->initializeOptimization();
    _so->optimize(_opts.iterations);

    /* Evaluate the quality of the map */
    double v = (*this)();
    timing.stop();

    _timing += timing.time();

    if(_opts.verbose) {
        fprintf(stderr, "iteration=%-3d phi=%-9.5lf value=%-9.5lf time=%-9.5lf cumTime=%-9.5lf\n",
                _iteration, phi, v, timing.time(), _timing);
    }

    _iteration++;

    /* Restore original kernel parameters */
    fforeach_map(g2o::OptimizableGraph::Edge *e, double d, previousKernelParams) {
        if(d == -INFINITY) {
            e->setRobustKernel(NULL);
        } else {
            e->robustKernel()->setDelta(d);
        }
    }

    /* Restore optimization algorithm */
    _so->setAlgorithm(wrapper.solver());

    /* Restore linearization point */
    _so->pop(allVertices);

    return v;
}

} /* namespace consist */

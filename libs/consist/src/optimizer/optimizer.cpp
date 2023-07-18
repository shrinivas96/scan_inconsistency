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

#include <signal.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/data/robot_laser.h>
#include <g2o/core/robust_kernel_impl.h>
#include "consist/gridbisectionoptimizer.h"
#include "consist/support.h"
#include "consist/mapquality.h"
#include "optimizeroptions.h"

G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(data)

void terminate(int)
{
    std::cerr << consist::support::cursorShow;
    exit(0);
}

using namespace consist;

int main(int argc, char **argv)
{
    signal(SIGINT, &terminate);

    OptimizerOptions args(argc, argv);
    if(args.abort) {
        return args.abortValue;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    typedef g2o::OptimizationAlgorithmGaussNewton OptimizationAlgorithm;

    g2o::SparseOptimizer *so = new g2o::SparseOptimizer();
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithm *solverAlg = new OptimizationAlgorithm(blockSolver);

    linearSolver->setBlockOrdering(false);
    solverAlg->setWriteDebug(false);
    so->setAlgorithm(solverAlg);
    so->load(args.fileName);
    so->setVerbose(args.g2oVerbose);
    so->vertex(0)->setFixed(true);

    MapQuality mq(so);
    mq.setOptions(args.mapQualityOptions());

    GridBisectionOptimizer gbo;
    gbo.setBounds(args.minphi, args.maxphi);
    gbo.setSteps(args.steps);
    gbo.setStepCriterion(
            args.linear ? GridBisectionOptimizer::Linear : GridBisectionOptimizer::Quadratic);
    gbo.setAcceptanceError(std::abs(args.optcond));

    double phi = gbo.optimize(mq);
    std::cout << "Optimal phi = " << phi << std::endl;

    delete so;

    return 0;
}

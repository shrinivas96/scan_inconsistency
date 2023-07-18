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

#include <g2o/core/factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <iostream>
#include <sstream>
#include <signal.h>
#include "consist/support.h"
#include "consist/hypothesistester.h"
#include "consist/inconsistencyse2.h"
#include "testeroptions.h"

#ifdef GUI_SUPPORT
#   include <QApplication>
#   include "testerwindow.h"
#endif /* GUI_SUPPORT */

G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(data)

using namespace consist;

void terminate(int)
{
    std::cerr << support::cursorShow;
    exit(0);
}

g2o::SparseOptimizer *load(const TesterOptions &opts, std::vector<InconsistencySE2> &tagged)
{
    const std::string inconsistencyTag("INCONSISTENCY_SE2");
    std::stringstream g2ostream;
    std::ifstream f(opts.fileName);
    while(f.good()) {
        std::string tag;
        std::stringstream line;
        g2o::readLine(f, line);
        line >> tag;
        if(tag.compare(inconsistencyTag) == 0) {
            InconsistencySE2 inc;
            inc.read(line);
            tagged.push_back(inc);
        } else {
            g2ostream << line.str() << std::endl;
        }
    }
    f.close();

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
    so->load(g2ostream);
    so->setVerbose(opts.g2oVerbose);
    so->vertex(0)->setFixed(true);

    if(opts.optimize > 0 && opts.dcs > -INFINITY) {
        fforeach(g2o::HyperGraph::Edge *e, so->edges()) {
            g2o::OptimizableGraph::Edge *oe = dynamic_cast<g2o::OptimizableGraph::Edge *>(e);
            if(oe) {
                oe->setRobustKernel(new g2o::RobustKernelDCS);
                oe->robustKernel()->setDelta(opts.dcs);
            }
        }
    }

    if(opts.optimize > 0) {
        so->initializeOptimization();
        so->optimize(opts.optimize);
    }

    return so;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, &terminate);

    TesterOptions args(argc, argv);
    if(args.abort) {
        return args.abortValue;
    }

    std::vector<InconsistencySE2> tagged;
    g2o::SparseOptimizer *so = load(args, tagged);
    HypothesisTester tester;
    tester.setOptions(args.hypothesisOptions);
    tester.setMap(so);
    delete so;

    tester.computeConsistencyMatrix();
    std::vector<int> badids = tester.binomialHypothesisTest();
    std::cout << "Inconsistent scan IDs: " << badids << std::endl;


#ifdef GUI_SUPPORT
    if(args.gui) {
        QApplication a(argc, argv);
        TesterWindow w;

        fforeach(int vertexid, badids) {
            w.addHighlight(vertexid, QColor::fromRgbF(1.0, 0.0, 0.0, 0.4));
        }

        w.setMap(tester.fullMap());
        w.setInconsistencies(tagged);
        w.displaySettingsChanged();
        w.show();

        return a.exec();
    } else {
        return 0;
    }
#else /* GUI_SUPPORT */
    return 0;
#endif /* GUI_SUPPORT */
}

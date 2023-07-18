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

#ifndef PARALLELRUNNER_H_
#define PARALLELRUNNER_H_

namespace consist {

/**
 * \brief Helper class for parallel function execution
 *
 * An instance of this class will spawn a set number of threads and execute
 * a function as many times as specified (with a different index parameter).
 * As soon as a function is done with the execution ParallelRunner will
 * schedule a new job, in order to maximize the processor usage at all times.
 *
 * The template paramenter \p ARG specifies the type of an additional
 * argument to pass to the function to parallelize.
 */
template <typename ARG = void>
class ParallelRunner {
public:
    /**
     * \brief Type of the function to parallelize.
     *
     * The function takes in two arguments. The first, an \c int, specifies
     * the index of the current job. The second, an \c ARG *, is a user
     * argument to pass to the function.
     */
    typedef void (*FunctionType)(int, ARG *);

    /**
     * \brief Type of the progress function.
     *
     * This is aimed to provide a verbose output of the progress. It takes
     * in two \c int arguments, the current job index and the total number of
     * jobs, and the user argument passed to the function.
     */
    typedef void (*ProgressFunctionType)(int, int, ARG *);

    /**
     * \brief Initialize the parallel runner.
     *
     * \param fun The function which will be parallelized
     */
    ParallelRunner(FunctionType fun);
    virtual ~ParallelRunner();

    /**
     * \brief Set the progress function.
     *
     * \param fun The function that will be used for reporting the progress.
     *            If set to NULL, no progress will be reported.
     */
    void setProgressFunction(ProgressFunctionType fun = NULL);

    /**
     * \brief Set the total number of jobs.
     *
     * \param jobs Number of jobs. Specifies how many times the FunctionType
     *             passed to the constructor will be executed.
     * \see ParallelRunner(FunctionType)
     */
    void setJobCount(int jobs);

    /**
     * \brief Run the parallel execution of the function.
     *
     * This method is blocking.
     *
     * \param argument Argument to pass to the function.
     * \param threads Number of threads to use for execution. If set to 0 the
     *                number will be chosen to be the logical number of
     *                processors of the system.
     */
    void run(ARG *argument, int threads = 0);

    /**
     * \brief Default progress function that will be used by ParallelRunner.
     *
     * This only prints on the stderr \"\p index/\p jobs\\r\".
     */
    static void defaultProgressFunction(int index, int jobs, ARG *argument);

private:
    struct ThreadContext {
        int index, jobs;
        pthread_mutex_t mutex;
        FunctionType function;
        ProgressFunctionType progress;
        ARG *argument;
    };

    static void *wrapper(void *arg);

    FunctionType _function;
    ProgressFunctionType _progress;
    int _jobs;
};

} /* namespace consist */

#include "parallelrunner.hpp"

#endif /* PARALLELRUNNER_H_ */

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

#include <pthread.h>
#include <Eigen/Core>

namespace consist {

template <typename ARG>
ParallelRunner<ARG>::ParallelRunner(ParallelRunner::FunctionType fun) :
        _function(fun), _progress(&ParallelRunner<ARG>::defaultProgressFunction)
{
}

template <typename ARG>
ParallelRunner<ARG>::~ParallelRunner()
{
}

template <typename ARG>
void ParallelRunner<ARG>::setJobCount(int jobs)
{
    _jobs = jobs;
}

template <typename ARG>
void ParallelRunner<ARG>::setProgressFunction(ProgressFunctionType fun)
{
    _progress = fun;
}

template <typename ARG>
void ParallelRunner<ARG>::run(ARG *argument, int threads)
{
    if(threads <= 0) {
        threads = sysconf(_SC_NPROCESSORS_ONLN);
    }

    pthread_t handles[threads];
    ThreadContext context;
    context.index    = 0;
    context.jobs     = _jobs;
    context.function = _function;
    context.progress = _progress;
    context.argument = argument;
    pthread_mutex_init(&context.mutex, NULL);

    for(int i = 0; i < threads; i++) {
        pthread_create(&handles[i], NULL, &ParallelRunner<ARG>::wrapper, &context);
    }

    for(int i = 0; i < threads; i++) {
        pthread_join(handles[i], NULL);
    }

    pthread_mutex_destroy(&context.mutex);
}

template <typename ARG>
void *ParallelRunner<ARG>::wrapper(void *arg)
{

    ThreadContext *context = static_cast<ThreadContext *>(arg);
    do {
        int index = -1;
        pthread_mutex_lock(&context->mutex);
        if(context->index < context->jobs) {
            index = context->index;
            context->index++;
        }
        if(context->progress && index >= 0) {
            context->progress(index, context->jobs, context->argument);
        }
        pthread_mutex_unlock(&context->mutex);

        if(index < 0) {
            return NULL;
        } else {
            context->function(index, context->argument);
        }
    } while(true);
    return NULL;
}

template <typename ARG>
void ParallelRunner<ARG>::defaultProgressFunction(int index, int jobs, ARG *)
{
    std::cerr << index + 1 << " / " << jobs << "\r" << std::flush;
}

} /* namespace consist */

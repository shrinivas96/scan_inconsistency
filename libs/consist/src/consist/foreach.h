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

#ifndef FOREACH_H_
#define FOREACH_H_

namespace consist {

/**
 * \def fforeach(decl, var)
 * Foreach macro implementation, slighly different from the one of Qt4 in that
 * it doesn't copy the contents of the container (doesn't work with temporaries).
 * Takes a variable definition  in \p decl and iterates over \p var.
 */
#define fforeach(decl, var) \
    for(::consist::__internal::ForeachContainer<__typeof__(var)> __f_container(var); \
        __f_container.b && __f_container.i != __f_container.e; \
        ++__f_container.i, __f_container.b--) \
        for(decl = *__f_container.i;; __extension__({ __f_container.b++; break; }))

/**
 * \def fforeach_map(declkey, declval, var)
 * Foreach macro implementation over std::map or any other class which
 * implements first and second in its iterator. Takes a variable definition
 * in \p declkey for the key and in \p declval for the value. Iterates over
 * \p var.
 */
#define fforeach_map(declkey, declval, var) \
    for(::consist::__internal::ForeachContainer<__typeof__(var)> __f_container(var); \
        __f_container.b && __f_container.i != __f_container.e; \
        ++__f_container.i, __f_container.b--) \
        for(declkey = __f_container.i->first ;; __extension__({ break; })) \
            for(declval = __f_container.i->second ;; __extension__({ __f_container.b++; break; }))

#ifndef CONSIST_PARSED_BY_DOXYGEN
namespace __internal {

template <typename T>
struct ForeachContainer {
    ForeachContainer(T &t) : b(1), i(t.begin()), e(t.end()) {}
    int b; typename T::iterator i, e;
};

template <typename T>
struct ForeachContainer<const T> {
    ForeachContainer(const T &t) : b(1), i(t.begin()), e(t.end()) {}
    int b; typename T::const_iterator i, e;
};

} /* namespace __internal */
#endif /* CONSIST_PARSED_BY_DOXYGEN */

} /* namespace consist */

#endif /* FOREACH_H_ */

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

namespace consist {

template <typename T, typename A>
std::ostream &operator<<(std::ostream &stream, const std::vector<T, A> &vec) {
    int i;
    stream << "{";
    for(i = 0; i < (int) vec.size() - 1; i++) {
        stream << vec[i] << ",";
    }
    if(i < (int) vec.size()) {
        stream << vec[i];
    }
    return stream << "}";
}

namespace support {

template <typename T>
const T *findFirstDatum(const g2o::HyperGraph::Vertex *v)
{
    const g2o::OptimizableGraph::Vertex *ov =
            dynamic_cast<const g2o::OptimizableGraph::Vertex *>(v);
    if(!ov) return NULL;

    const g2o::OptimizableGraph::Data *d = ov->userData();
    while(d && !dynamic_cast<const T *>(d)) {
        d = d->next();
    }
    return dynamic_cast<const T *>(d);
}

template <typename T>
T *findFirstDatum(g2o::HyperGraph::Vertex *v)
{
    /* Maybe do const_cast to avoid copypasting code? */
    g2o::OptimizableGraph::Vertex *ov =
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(v);
    if(!ov) return NULL;

    g2o::OptimizableGraph::Data *d = ov->userData();
    while(d && !dynamic_cast<T *>(d)) {
        d = d->next();
    }
    return dynamic_cast<T *>(d);
}

template <typename Container>
typename Container::value_type sum(const Container &c) {
    typename Container::value_type t = 0;
    fforeach(const typename Container::value_type &v, c) {
        t += v;
    }
    return t;
}

template <typename Container>
typename Container::value_type mean(const Container &c) {
    int n = 0;
    typename Container::value_type t = 0;
    fforeach(const typename Container::value_type &v, c) {
        t += v;
        n++;
    }

    if(n > 0) {
        return t / n;
    } else {
        return 0;
    }
}

template <typename Container>
typename Container::value_type variance(const Container &c) {
    int n = -1;
    typename Container::value_type t = 0, m = mean(c);
    fforeach(const typename Container::value_type &v, c) {
        t += (v - m) * (v - m);
        n++;
    }
    if(n > 0) {
        return t / n;
    } else {
        return 0;
    }
}

} /* namespace support */

} /* namespace consist */

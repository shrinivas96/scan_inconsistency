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

#include "mapviewer.h"
#include <QKeyEvent>
#include <QMouseEvent>
#include <g2o/types/data/raw_laser.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include "consist/visibility.h"
#include "consist/foreach.h"
#include "consist/support.h"

using namespace consist;
using namespace consist::support;

MapViewer::MapViewer(QWidget *parent) :
    QGLViewer(parent), _so(NULL), _scan(0), _dragging(false)
{
}

MapViewer::~MapViewer()
{
}

int MapViewer::scan() const
{
    return std::lower_bound(_indices.begin(), _indices.end(), _scan) - _indices.begin();
}

int MapViewer::scanCount() const
{
    return _indices.size();
}

Point MapViewer::imageToWorldCoordinates(double x, double y) const
{
    double halfRangeX = camera()->position().z * std::tan(camera()->horizontalFieldOfView() / 2);
    double halfRangeY = camera()->position().z * std::tan(camera()->fieldOfView() / 2);
    double angle = 2 * std::atan2(camera()->orientation()[2], camera()->orientation()[3]);
    g2o::SE2 rt(camera()->position().x, camera()->position().y, angle);

    double xcoord = halfRangeX * (2 * x / camera()->screenWidth() - 1);
    double ycoord = halfRangeY * (1 - 2 * y / camera()->screenHeight());

    return rt * Point(xcoord, ycoord);
}

void MapViewer::draw()
{
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);

    if(_so) {
        drawHighlights();
        drawInconsistencies();
        drawScans();
        drawPoseGraph();
    }
}

void MapViewer::drawArrowhead(const g2o::SE2 &where, double base, double height)
{
    /* Assumes glBegin has already been called */
    Point a(0, - base / 2), b(0, base / 2), c(height, 0);
    a = where * a;
    b = where * b;
    c = where * c;
    glVertex2f(a.x(), a.y());
    glVertex2f(b.x(), b.y());
    glVertex2f(c.x(), c.y());
}

void MapViewer::drawScans()
{
    if(_conf.drawScans) {

        glColor3f(0, 0, 0);

        if(_conf.drawScansAsPolylines) {
            glBegin(GL_LINES);
            fforeach_map(int id, const g2o::HyperGraph::Vertex *v, _so->vertices()) {
                const Visibility *vis = findFirstDatum<Visibility>(
                        static_cast<const g2o::OptimizableGraph::Vertex *>(v));
                if(vis) {
                    const PointArray &points = vis->points();
                    const std::vector<Visibility::FrontierType> &edges = vis->edgetypes();
                    for(size_t i = 0; i < edges.size(); i++) {
                        if(edges[i] == Visibility::Obstacle) {
                            glVertex2f(points[i    ].x(), points[i    ].y());
                            glVertex2f(points[i + 1].x(), points[i + 1].y());
                        }
                    }
                }
            }
            glEnd();
        } else {
            glBegin(GL_POINTS);
            fforeach_map(int id, const g2o::HyperGraph::Vertex *v, _so->vertices()) {
                const g2o::RawLaser *scan = findFirstDatum<g2o::RawLaser>(
                        static_cast<const g2o::OptimizableGraph::Vertex *>(v));
                if(scan) {
                    PointArray points = static_cast<const g2o::VertexSE2 *>(v)->estimate() * scan->cartesian();
                    for(size_t i = 0; i < points.size(); i++) {
                        glVertex2f(points[i].x(), points[i].y());
                    }
                }
            }
            glEnd();
        }

        if(_conf.drawVisibilityRays) {
            glBegin(GL_LINES);
            glColor4f(0, 0, 1, 0.4);
            const Visibility *vis = findFirstDatum<Visibility>(
                    static_cast<const g2o::OptimizableGraph::Vertex *>(_so->vertex(_scan)));
            if(vis) {
                fforeach(const Point &ray, vis->rays()) {
                    glVertex2f(vis->viewPoint().x(), vis->viewPoint().y());
                    glVertex2f(ray.x(), ray.y());
                }
            }
            glEnd();
        }
    }
}

void MapViewer::drawPoseGraph()
{
    if(_conf.drawPoseGraph) {
        glColor3f(0, 0, 1);

        int i = 0;
        glBegin(GL_TRIANGLES);
        fforeach_map(int id, const g2o::HyperGraph::Vertex *v, _so->vertices()) {
            const g2o::VertexSE2 *vse2 = dynamic_cast<const g2o::VertexSE2 *>(v);
            if(vse2) {
                drawArrowhead(vse2->estimate(), 0.15, 0.22);
            }
        }
        glEnd();


        glBegin(GL_LINES);
        fforeach(const g2o::HyperGraph::Edge *e, _so->edges()) {
            const g2o::EdgeSE2 *ese2 = dynamic_cast<const g2o::EdgeSE2 *>(e);
            if(ese2) {
                const g2o::VertexSE2 *v0 = dynamic_cast<const g2o::VertexSE2 *>(ese2->vertex(0));
                const g2o::VertexSE2 *v1 = dynamic_cast<const g2o::VertexSE2 *>(ese2->vertex(1));
                if(v0 && v1) {
                    glVertex2f(v0->estimate().translation().x(), v0->estimate().translation().y());
                    glVertex2f(v1->estimate().translation().x(), v1->estimate().translation().y());
                }
            }
        }
        glEnd();
    }
}

void MapViewer::drawQuadrilateral(const Quadrilateral &q, float r, float g, float b)
{
    glBegin(GL_TRIANGLE_STRIP);
    glColor4f(r, g, b, 0.4);
    glVertex2f(q.p1().x(), q.p1().y());
    glVertex2f(q.p2().x(), q.p2().y());
    glVertex2f(q.p4().x(), q.p4().y());
    glVertex2f(q.p3().x(), q.p3().y());
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(r, g, b);
    glVertex2f(q.p1().x(), q.p1().y());
    glVertex2f(q.p2().x(), q.p2().y());
    glVertex2f(q.p3().x(), q.p3().y());
    glVertex2f(q.p4().x(), q.p4().y());
    glEnd();
}

void MapViewer::drawInconsistencies()
{
    if(!_conf.drawInconsistencies) {
        return;
    }

    for(int i = 0; i < _inconsistencies.size(); i++) {
        if(_inconsistencies[i].type() == InconsistencySE2::Strong) {
            drawQuadrilateral(_inconsistencies[i].region(), 1, 0, 0);
        } else {
            drawQuadrilateral(_inconsistencies[i].region(), 0, 1, 0);
        }
    }

    if(_dragging) {
        Point p1 = imageToWorldCoordinates(_fromx, _fromy);
        Point p2 = imageToWorldCoordinates(_tox,   _fromy);
        Point p3 = imageToWorldCoordinates(_tox,   _toy);
        Point p4 = imageToWorldCoordinates(_fromx, _toy);
        Quadrilateral q(p1, p2, p3, p4);

        if(_currenttype == InconsistencySE2::Strong) {
            drawQuadrilateral(q, 1, 0, 0);
        } else {
            drawQuadrilateral(q, 0, 1, 0);
        }
    }

}

void MapViewer::drawHighlights()
{
    if(_conf.drawHighlights) {
        for(size_t i = 0; i < _highlights.size(); i++) {
            const Visibility *v = findFirstDatum<Visibility>(_so->vertex(_highlights[i]));
            const QColor &color = _highlightcolors[i];

            glColor4f(color.redF(), color.greenF(), color.blueF(), color.alphaF());
            glBegin(GL_TRIANGLE_FAN);

            if(v) {
                glVertex2f(v->viewPoint().x(), v->viewPoint().y());
                fforeach(const Point &p, v->points()) {
                    glVertex2f(p.x(), p.y());
                }
            }

            glEnd();
        }
    }
}


void MapViewer::addHighlight(int vertexid, const QColor &color)
{
    _highlights.push_back(vertexid);
    _highlightcolors.push_back(color);
    //updateGL();
}

void MapViewer::setConfiguration(const MapViewerConfiguration &conf)
{
    _conf = conf;
    updateGL();
}

const MapViewerConfiguration &MapViewer::configuration() const
{
    return _conf;
}


void MapViewer::init()
{
  setShortcut(EXIT_VIEWER, Qt::CTRL + Qt::Key_Q);
  setShortcut(FULL_SCREEN, Qt::CTRL + Qt::Key_F);
  setShortcut(DISPLAY_FPS, 0);
  setShortcut(DRAW_GRID, 0);
  setShortcut(DRAW_AXIS, 0);
  setShortcut(ENABLE_TEXT, 0);
  setShortcut(SAVE_SCREENSHOT, 0);
  setShortcut(CAMERA_MODE, 0);
  setShortcut(STEREO, 0);
  setShortcut(ANIMATION, 0);
  setShortcut(HELP, 0);
  setShortcut(EDIT_CAMERA, 0);
  setShortcut(MOVE_CAMERA_LEFT, 0);
  setShortcut(MOVE_CAMERA_RIGHT, 0);
  setShortcut(MOVE_CAMERA_UP, 0);
  setShortcut(MOVE_CAMERA_DOWN, 0);

  setMouseBinding(Qt::RightButton, CAMERA, ROLL);
  setMouseBinding(Qt::LeftButton, CAMERA, TRANSLATE);

  setWheelBinding(Qt::NoModifier, CAMERA, ZOOM);

  camera()->setOrientation(qglviewer::Quaternion(0, 0, 0, 1));
  camera()->setPosition(qglviewer::Vec(0, 0, 30));
}

void MapViewer::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_Z && e->modifiers() == Qt::ControlModifier) {
        if(!_inconsistencies.empty()) {
            _inconsistencies.pop_back();
            updateGL();
        }
    } else {
        QGLViewer::keyPressEvent(e);
    }
}

void MapViewer::mousePressEvent(QMouseEvent* e)
{
    if(_conf.interactive && e->button() == Qt::LeftButton && e->modifiers() != Qt::NoModifier) {
        _fromx = _tox = e->x();
        _fromy = _toy = e->y();
        if(e->modifiers() == Qt::ControlModifier) {
            _dragging = true;
            _currenttype = InconsistencySE2::Strong;
        } else if(e->modifiers() == Qt::ShiftModifier) {
            _dragging = true;
            _currenttype = InconsistencySE2::Weak;
        }
    } else {
        QGLViewer::mousePressEvent(e);
    }
}

void MapViewer::mouseMoveEvent(QMouseEvent* e)
{
    if(_dragging) {
        _tox = e->x();
        _toy = e->y();
        updateGL();
    } else {
        QGLViewer::mouseMoveEvent(e);
    }
}

void MapViewer::mouseReleaseEvent(QMouseEvent* e)
{
    if(_dragging) {
        Point p1 = imageToWorldCoordinates(_fromx, _fromy);
        Point p2 = imageToWorldCoordinates(_tox,   _fromy);
        Point p3 = imageToWorldCoordinates(_tox,   _toy);
        Point p4 = imageToWorldCoordinates(_fromx, _toy);
        Quadrilateral q(p1, p2, p3, p4);
        _inconsistencies.push_back(InconsistencySE2(q, _currenttype));
        _dragging = false;
        updateGL();
    } else {
        QGLViewer::mouseReleaseEvent(e);
    }
}

void MapViewer::setMap(const g2o::SparseOptimizer *so)
{
    _so = so;
    _indices.clear();
    if(_so) {
        fforeach_map(int id, g2o::HyperGraph::Vertex *v, _so->vertices()) {
            if(findFirstDatum<g2o::RawLaser>(v)) {
                _indices.push_back(id);
            }
        }
        std::sort(_indices.begin(), _indices.end());
        _scan = _indices.front();
    }
}

void MapViewer::setInconsistencies(const std::vector<InconsistencySE2> &inconsistencies)
{
    _inconsistencies = inconsistencies;
}

void MapViewer::setScan(int n)
{
    if(n >= 0 && n < _indices.size()) {
        int prevscan = _scan;
        _scan = _indices[n];
        if(_scan != prevscan) {
            /*
            qglviewer::Vec pos = camera()->position();
            const Visibility &vis = _scans->at(_scan).visibility();
            pos.x = vis.viewPoint().x();
            pos.y = vis.viewPoint().y();
            camera()->setPosition(pos);
            updateGL();
            */
            emit scanChanged(n);
        }
    }
}

const std::vector<InconsistencySE2> &MapViewer::inconsistencies() const
{
    return _inconsistencies;
}

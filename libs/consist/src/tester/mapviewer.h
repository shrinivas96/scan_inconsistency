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

#ifndef MAPVIEWER_H_
#define MAPVIEWER_H_

#include <QObject>
#include <QGLViewer/qglviewer.h>
#include <g2o/core/sparse_optimizer.h>
#include <vector>
#include "consist/inconsistencyse2.h"


struct MapViewerConfiguration {
    bool drawScans;
    bool drawScansAsPolylines;
    bool drawVisibilityRays;
    bool drawPoseGraph;
    bool drawInconsistencies;
    bool drawHighlights;
    bool interactive;

    MapViewerConfiguration() :
        drawScans(true), drawScansAsPolylines(false), drawVisibilityRays(true),
        drawPoseGraph(true), drawInconsistencies(true), drawHighlights(true),
        interactive(false) {}
};

class MapViewer : public QGLViewer
{
    Q_OBJECT

public:
    MapViewer(QWidget *parent = NULL);
    virtual ~MapViewer();
    void setMap(const g2o::SparseOptimizer *map);
    void setInconsistencies(
            const std::vector<consist::InconsistencySE2> &inconsistencies);
    void addHighlight(int vertexid, const QColor &color);

    void setConfiguration(const MapViewerConfiguration &conf);
    const MapViewerConfiguration &configuration() const;

    const std::vector<consist::InconsistencySE2> &inconsistencies() const;

    int scan() const;
    int scanCount() const;

signals:
    void scanChanged(int scan);

public slots:
    void setScan(int n);

protected:
    virtual void draw();
    virtual void init();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

    void drawArrowhead(const g2o::SE2 &where, double base, double height);

    void drawScans();
    void drawPoseGraph();
    void drawInconsistencies();
    void drawHighlights();

    void drawQuadrilateral(const consist::Quadrilateral &q, float r, float g, float b);

    consist::Point imageToWorldCoordinates(double x, double y) const;

private:
    MapViewerConfiguration _conf;
    const g2o::SparseOptimizer *_so;
    std::vector<int> _highlights;
    std::vector<QColor> _highlightcolors;
    std::vector<int> _indices;
    int _scan;
    bool _dragging;

    std::vector<consist::InconsistencySE2> _inconsistencies;
    consist::InconsistencySE2::InconsistencyType _currenttype;
    int _fromx, _fromy;
    int _tox, _toy;
};

#endif /* MAPVIEWER_H_ */

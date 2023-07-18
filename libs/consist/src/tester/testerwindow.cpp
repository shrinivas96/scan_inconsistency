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

#include "testerwindow.h"
#include "ui_testerwindow.h"
#include <g2o/core/sparse_optimizer.h>
#include <QKeyEvent>
#include <QFileDialog>
#include <fstream>

TesterWindow::TesterWindow(QWidget *parent) :
    QMainWindow(parent),
    _ui(new Ui::TesterWindow), _so(NULL)
{
    _ui->setupUi(this);
    _ui->usageText->viewport()->setAutoFillBackground(false);
    connect(_ui->showHighlights,      SIGNAL(clicked()), this, SLOT(displaySettingsChanged()));
    connect(_ui->showInconsistencies, SIGNAL(clicked()), this, SLOT(displaySettingsChanged()));
    connect(_ui->showPoseGraph,       SIGNAL(clicked()), this, SLOT(displaySettingsChanged()));
    connect(_ui->showScans,           SIGNAL(clicked()), this, SLOT(displaySettingsChanged()));
    connect(_ui->showPolyScans,       SIGNAL(clicked()), this, SLOT(displaySettingsChanged()));
    connect(_ui->showVisibilityRays,  SIGNAL(clicked()), this, SLOT(displaySettingsChanged()));
    connect(_ui->save,                SIGNAL(clicked()), this, SLOT(saveClicked()));
    connect(_ui->scan,                SIGNAL(valueChanged(int)), this, SLOT(scanChanged(int)));
}

TesterWindow::~TesterWindow()
{
    delete _ui;
}

MapViewerConfiguration TesterWindow::configuration()
{
    MapViewerConfiguration conf;
    conf.interactive          = true;
    conf.drawHighlights       = _ui->showHighlights->checkState()      == Qt::Checked;
    conf.drawInconsistencies  = _ui->showInconsistencies->checkState() == Qt::Checked;
    conf.drawPoseGraph        = _ui->showPoseGraph->checkState()       == Qt::Checked;
    conf.drawScans            = _ui->showScans->checkState()           == Qt::Checked;
    conf.drawScansAsPolylines = _ui->showPolyScans->checkState()       == Qt::Checked;
    conf.drawVisibilityRays   = _ui->showVisibilityRays->checkState()  == Qt::Checked;
    return conf;
}

void TesterWindow::setInconsistencies(
        const std::vector<consist::InconsistencySE2> &inconsistencies)
{
    _ui->viewer->setInconsistencies(inconsistencies);
}

void TesterWindow::setMap(g2o::SparseOptimizer *so)
{
    _so = so;
    if(_so) {
        _ui->viewer->setMap(_so);
        _ui->nscans->setText(QString::number(_ui->viewer->scanCount()));
        _ui->scan->setMaximum(_ui->viewer->scanCount());
        _ui->scan->setMinimum(1);
        _ui->scan->setValue(1);
    }
}

void TesterWindow::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_Up) {
        scanChanged(_ui->viewer->scan() + 2);
    } else if(e->key() == Qt::Key_Down) {
        scanChanged(_ui->viewer->scan());
    }
}

void TesterWindow::addHighlight(int vertexid, const QColor &color)
{
    _ui->viewer->addHighlight(vertexid, color);
}

void TesterWindow::displaySettingsChanged()
{
    _ui->viewer->setConfiguration(configuration());
}

void TesterWindow::scanChanged(int scan)
{
    _ui->viewer->setScan(scan - 1);
    _ui->viewer->updateGL();
    _ui->scan->setValue(_ui->viewer->scan() + 1);
}

void TesterWindow::saveClicked()
{
    QString fname = QFileDialog::getSaveFileName(
                this, "Save map with tagged inconsistencies", "",
                "gm2dl file (*.gm2dl);;");
    if(fname.size() > 0) {
        if(!fname.contains(".")) {
            fname += ".gm2dl";
        }
        std::ofstream f(fname.toLocal8Bit().data());
        _so->save(f);
        fforeach(const consist::InconsistencySE2 &inc,
                _ui->viewer->inconsistencies()) {
            f << "INCONSISTENCY_SE2 ";
            inc.write(f);
            f << std::endl;
        }
        f.close();
    }
}

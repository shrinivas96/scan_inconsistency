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

#ifndef TESTERWINDOW_H_
#define TESTERWINDOW_H_

#include <QMainWindow>
#include "mapviewer.h"
#include "consist/inconsistencyse2.h"

namespace g2o { class SparseOptimizer; }
namespace Ui  { class TesterWindow;    }

class TesterWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit TesterWindow(QWidget *parent = 0);
    ~TesterWindow();
    void setMap(g2o::SparseOptimizer *so);
    void addHighlight(int vertexid, const QColor &color);
    void setInconsistencies(const std::vector<consist::InconsistencySE2> &inconsistencies);
    
    MapViewerConfiguration configuration();

public slots:
    void displaySettingsChanged();

private slots:
    void scanChanged(int scan);
    void saveClicked();

protected:
    void keyPressEvent(QKeyEvent *e);

private:
    Ui::TesterWindow *_ui;
    g2o::SparseOptimizer *_so;
};

#endif /* TESTERWINDOW_H_ */

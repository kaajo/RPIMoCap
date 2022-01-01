/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2019 Miroslav Krajicek.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Server/mainwindow.h"
#include "Server/rpimocapserver.h"
#include "Server/calibrationwidget.h"
#include "Server/floorcalibrationwidget.h"

#include <QApplication>
#include <QtWidgets/QApplication>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QApplication::setOrganizationName("Miroslav Krajicek");
    QApplication::setApplicationVersion("0.1");
    QApplication::setApplicationName("RPIMoCap");

    qSetMessagePattern("%{type} %{if-category}%{category}: %{endif}%{message}");

    qRegisterMetaType<std::vector<RPIMoCap::Line3D>>("std::vector<RPIMoCap::Line3D>");
    qRegisterMetaType<RPIMoCap::Line3D>("RPIMoCap::Line3D");

    RPIMoCap::Server server;
    RPIMoCap::MainWindow w;

    QObject::connect(&w, &RPIMoCap::MainWindow::searchForCameras, &server, &RPIMoCap::Server::init);
    QObject::connect(&w,&RPIMoCap::MainWindow::startMoCap, &server, &RPIMoCap::Server::onMoCapStart);
    QObject::connect(w.calibrationWidget(),&RPIMoCap::CalibrationWidget::startCalibration,
                     &server, &RPIMoCap::Server::onCalibStart);
    QObject::connect(w.floorCalibration(), &RPIMoCap::FloorCalibrationWidget::snapshotRequest,
                     &server, &RPIMoCap::Server::calibrateFloor);
    QObject::connect(&server, &RPIMoCap::Server::cameraAdded, &w, &RPIMoCap::MainWindow::addCamera);
    QObject::connect(&server, &RPIMoCap::Server::cameraRemoved, &w, &RPIMoCap::MainWindow::removeCamera);
    QObject::connect(&server, &RPIMoCap::Server::frameReady, &w, &RPIMoCap::MainWindow::drawFrame);

    w.show();
    return QApplication::exec();
}

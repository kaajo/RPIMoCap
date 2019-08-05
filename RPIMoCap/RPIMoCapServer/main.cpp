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

#include "mainwindow.h"
#include "linesaggregator.h"

#include <mqttpublisher.h>
#include <mqttsubscriber.h>
#include <line3d.h>

#include <QApplication>
#include <QtWidgets/QApplication>
#include <QDebug>

#include <memory>

#include <QDir>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QApplication::setOrganizationName("Miroslav Krajicek");
    QApplication::setApplicationVersion("1.0");
    QApplication::setApplicationName("RPIMoCap");

    qRegisterMetaType<QVector<RPIMoCap::Line3D>>("QVector<RPIMoCap::Line3D>");
    qRegisterMetaType<RPIMoCap::Line3D>("RPIMoCap::Line3D");

    RPIMoCap::MQTTSettings settings;
    settings.IPAddress = "127.0.0.1";

    RPIMoCap::MQTTPublisher publisher("serverTriggerPub", "/trigger", settings);
    RPIMoCap::MQTTSubscriber subscriber("serverLinesSub","/lines",settings);

    LinesAggregator aggregator;
    QObject::connect(&subscriber,&RPIMoCap::MQTTSubscriber::messageReceived,&aggregator,&LinesAggregator::onLinesReceived);
    QObject::connect(&aggregator,&LinesAggregator::trigger,&publisher,&RPIMoCap::MQTTPublisher::publishData);


    publisher.publishData({});

    return a.exec();
}

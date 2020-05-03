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

#include <RPIMoCap/ClientLib/client.h>
#include <RPIMoCap/ClientLib/rpicamera.h>

#include <QCoreApplication>

#include <csignal>
#include <iostream>

constexpr float degToRad = M_PI/180.0f;

std::unique_ptr<RPIMoCap::Client> client;

void signal_handler(int signal)
{
    std::cout << "reset" << std::endl;
    client.reset(nullptr);
}

int main(int argc, char *argv[])
{
    mosqpp::lib_init();

    std::signal(SIGINT | SIGTERM, signal_handler);

    QCoreApplication a(argc, argv);

    qSetMessagePattern("%{type} %{if-category}%{category}: %{endif}%{message}");

    RPIMoCap::Camera::Intrinsics params = RPIMoCap::Camera::Intrinsics::computeRPICameraV2Params();

    auto camera = std::make_shared<RPIMoCap::GstCVCamera>("v4l2src device=/dev/video0 ! "
                                                "video/x-raw,width=640,height=480,framerate=90/1 ! "
                                                "videoconvert ! video/x-raw,format=GRAY8 ! "
                                                "appsink max-buffers=1 name=appsink");

    QUuid id = QUuid::createUuid();

    client.reset(new RPIMoCap::Client(camera, params, id));
    return a.exec();
}

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

#include "RPIMoCap/SimClient/simcamera.h"
#include "RPIMoCap/SimClient/mainwindow.h"

#include <RPIMoCap/ClientLib/rpimocapclient.h>

#include <QApplication>

constexpr float degToRad = M_PI/180.0f;

//diag 64.92 https://www.raspberrypi.org/documentation/hardware/camera/
RPIMoCap::CameraParams computeRPICameraV1Params(const cv::Size2f fullFoVRad = cv::Size2f(53.5f * degToRad, 41.41f * degToRad))
{
    const cv::Size2i fullResolution(2592, 1944);

    RPIMoCap::CameraParams params;
    params.imageSize = cv::Size2i(640, 480);
    params.cameraMatrix.at<float>(0,0) = params.imageSize.width / tanf(fullFoVRad.width/2.0f);
    params.cameraMatrix.at<float>(1,1) = params.imageSize.height / tanf(fullFoVRad.height/2.0f);
    params.cameraMatrix.at<float>(0,2) = 320;
    params.cameraMatrix.at<float>(1,2) = 240;

    return params;
}

//https://elinux.org/Rpi_Camera_Module#Technical_Parameters_.28v.2_board.29
RPIMoCap::CameraParams computeRPICameraV2Params(const cv::Size2f fullFoVRad = cv::Size2f(62.2f * degToRad,48.8f * degToRad))
{
    //https://www.raspberrypi.org/forums/viewtopic.php?t=157384
    const cv::Size2i fullResolution(3280, 2464);
    const cv::Size2i binnedResolution(1280, 960);

    const int leftRightCrop = 1000;
    const int topBotCrop = 752;

    assert(binnedResolution.width + 2 * leftRightCrop == fullResolution.width);
    assert(binnedResolution.height + 2 * topBotCrop == fullResolution.height);

    const float croppedFoVWidth = binnedResolution.width*fullFoVRad.width/fullResolution.width;
    const float croppedFoVHeight = binnedResolution.height*fullFoVRad.height/fullResolution.height;

    RPIMoCap::CameraParams params;
    params.imageSize = cv::Size2i(640, 480);
    params.cameraMatrix.at<float>(0,0) = params.imageSize.width / tanf(croppedFoVWidth/2.0f);
    params.cameraMatrix.at<float>(1,1) = params.imageSize.height / tanf(croppedFoVHeight/2.0f);
    params.cameraMatrix.at<float>(0,2) = 320;
    params.cameraMatrix.at<float>(1,2) = 240;

    return params;
}

int main(int argc, char *argv[])
{
    mosqpp::lib_init();

    QApplication a(argc, argv);

    qSetMessagePattern("%{type} %{if-category}%{category}: %{endif}%{message}");

    RPIMoCap::CameraParams params = computeRPICameraV1Params();

    RPIMoCap::SimClient::SimScene scene;

    auto *mainWindow = new RPIMoCap::SimClient::MainWindow(scene);

    RPIMoCap::SimClient::SimScene::Marker marker;
    marker.translation = cv::Point3f(0,0,10);
    scene.setMarkers({marker});

    auto camera = std::make_shared<RPIMoCap::SimClient::SimCamera>(params, scene);

    RPIMoCapClient client(camera, params);

    mainWindow->show();
    return a.exec();
}

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

#pragma once

#include <opencv2/core/mat.hpp>

#include <QVariantMap>

#include <assert.h>

namespace RPIMoCap::Camera {

constexpr float degToRad = M_PI/180.0f;

struct Intrinsics
{
    uint32_t maxFPS = 0;
    cv::Size imageSize = cv::Size(0,0);
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat cameraMatrixInv = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat distortionCoeffs = cv::Mat(1, 4, CV_32FC1, cv::Scalar(0.0f));

    QVariantMap toVariantMap() const
    {
        QVariantMap data;
        data["imageWidth"] = imageSize.width;
        data["imageHeight"] = imageSize.height;
        data["maxFPS"] = maxFPS;
        data["fx"] = cameraMatrix.at<float>(0,0);
        data["fy"] = cameraMatrix.at<float>(1,1);
        data["cx"] = cameraMatrix.at<float>(0,2);
        data["cy"] = cameraMatrix.at<float>(1,2);

        for (size_t i = 0; i < distortionCoeffs.cols; ++i)
        {
            data["dist" + QString::number(i)] = distortionCoeffs.at<float>(0,i);
        }

        return data;
    }

    static Intrinsics fromVariantMap(const QVariantMap &varMap)
    {
        Intrinsics params;
        params.imageSize = cv::Size(varMap["imageWidth"].toUInt(),
                                    varMap["imageHeight"].toUInt());
        params.maxFPS = varMap["maxFPS"].toUInt();
        params.cameraMatrix.at<float>(0,0) = varMap["fx"].toDouble();
        params.cameraMatrix.at<float>(1,1) = varMap["fy"].toDouble();
        params.cameraMatrix.at<float>(0,2) = varMap["cx"].toDouble();
        params.cameraMatrix.at<float>(1,2) = varMap["cy"].toDouble();

        //TODO distortion

        params.cameraMatrixInv = params.cameraMatrix.inv();

        return params;
    }

    //diag 64.92 https://www.raspberrypi.org/documentation/hardware/camera/
    static Intrinsics computeRPICameraV1Params(const cv::Size2f fullFoVRad = cv::Size2f(53.5f * degToRad, 41.41f * degToRad))
    {
        const cv::Size2i fullResolution(2592, 1944);

        Intrinsics params;
        params.maxFPS = 90;
        params.imageSize = cv::Size2i(640, 480);
        params.cameraMatrix.at<float>(0,0) = params.imageSize.width / tanf(fullFoVRad.width/2.0f);
        params.cameraMatrix.at<float>(1,1) = params.imageSize.height / tanf(fullFoVRad.height/2.0f);
        params.cameraMatrix.at<float>(0,2) = 320;
        params.cameraMatrix.at<float>(1,2) = 240;
        params.cameraMatrixInv = params.cameraMatrix.inv();

        return params;
    }

    //https://elinux.org/Rpi_Camera_Module#Technical_Parameters_.28v.2_board.29
    static Intrinsics computeRPICameraV2Params(const cv::Size2f fullFoVRad = cv::Size2f(62.2f * degToRad,48.8f * degToRad))
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

        Intrinsics params;
        params.maxFPS = 90;
        params.imageSize = cv::Size2i(640, 480);
        params.cameraMatrix.at<float>(0,0) = params.imageSize.width / tanf(croppedFoVWidth/2.0f);
        params.cameraMatrix.at<float>(1,1) = params.imageSize.height / tanf(croppedFoVHeight/2.0f);
        params.cameraMatrix.at<float>(0,2) = 320;
        params.cameraMatrix.at<float>(1,2) = 240;
        params.cameraMatrixInv = params.cameraMatrix.inv();

        return params;
    }
};

}

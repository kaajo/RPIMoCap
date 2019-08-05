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

#include <QObject>
#include <QDebug>

#include <opencv2/core/mat.hpp>
#include <gst/gstelement.h>
#include <gst/app/app.h>

class GstCVCamera {
public:
    GstCVCamera(std::string pipelineDescription);

    ~GstCVCamera();

    GstCVCamera(const GstCVCamera&) = delete;
    void operator=(const GstCVCamera&) = delete;

    bool open();

    void close();

    constexpr static const char* const appSinkName = "appsink";

    cv::Mat pullData();
private:
    bool init();

    static cv::Size2i getWidthHeightFromCaps(GstCaps* gstCaps);
    bool changePipelineState(const GstState& state);

    cv::Mat preprocessData(GstBuffer*, const GstMapInfo& mappedBuffer, GstCaps* bufferCaps);

    GstAppSink* appsink_ = nullptr;

    std::string pipelineDescription_;

    GstElement* pipeline_ = nullptr;
    GstState lastDesiredState_ = GST_STATE_PAUSED;

    bool m_opened = false;
};

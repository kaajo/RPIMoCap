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

#include "RPIMoCap/ClientLib/rpicamera.h"

#include <QDebug>

#include <chrono>

namespace RPIMoCap {

GstCVCamera::GstCVCamera(std::__cxx11::string pipelineDescription)
    : pipelineDescription_(pipelineDescription)
{

}

GstCVCamera::~GstCVCamera() {
    close();
}

bool GstCVCamera::open() {
    if (m_opened) {
        return true;
    }

    if (!init()) {
        return false;
    }

    m_opened = changePipelineState(GstState::GST_STATE_PLAYING);

    return m_opened;
}

void GstCVCamera::close() {
    if (!m_opened) {
        return;
    }

    changePipelineState(GST_STATE_NULL);
    gst_object_unref(appsink_);
    gst_object_unref(pipeline_);

    m_opened = false;
}

cv::Mat GstCVCamera::pullData() {
    GstSample* const sample = gst_app_sink_try_pull_sample(appsink_, 1000000000);

    if (!sample) {
        qDebug() << "cannot obtain sample from sink.";
        return cv::Mat();
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);

    if (!buffer) {
        qDebug() << ": failed to get buffer from sample";
        return cv::Mat();
    }

    GstMapInfo mappedBuffer;
    if (!gst_buffer_map(buffer, &mappedBuffer, GST_MAP_READ)) {
        qDebug() << ": failed to map a buffer";
        return cv::Mat();
    }

    const void* mapData = mappedBuffer.data;
    const gsize mapSize = mappedBuffer.size;
    if (!mapData || mapSize <= 0) {
        gst_buffer_unmap(buffer, &mappedBuffer);
        gst_sample_unref(sample);
        qDebug() << "mapped data is null";
        return cv::Mat();
    }

    GstCaps* caps = gst_sample_get_caps(sample);

    const cv::Mat data = preprocessData(buffer, mappedBuffer, caps);

    gst_buffer_unmap(buffer, &mappedBuffer);
    gst_sample_unref(sample);

    return data;
}

bool GstCVCamera::init() {
    GError* error = nullptr;

    if (!gst_init_check(nullptr, nullptr, &error)) {
        qDebug() << QString::fromStdString( "GStreamer couldn't be initialized: " + std::string(error->message));
        return false;
    }

    pipeline_ = gst_parse_launch(pipelineDescription_.c_str(), &error);

    if (error != nullptr) {
        qDebug() << QString::fromStdString( "GStreamer parse-pipeline error: " + std::string(error->message));
        return false;
    }

    qDebug() << QString::fromStdString( "GStreamer: gst_parse_launch successful for streaming pipeline: " + pipelineDescription_);

    GstElement* tmpsink = gst_bin_get_by_name(GST_BIN(pipeline_), appSinkName);

    if (!tmpsink) {
        qDebug() << "Failed to acquire appsink under the name '" + QString(appSinkName) + "'";
        return false;
    }

    if (!GST_IS_APP_SINK(tmpsink)) {
        gst_object_unref(tmpsink);
        qDebug() << "The '" + QString(appSinkName) + "' element is not an appsink";
        return false;
    }

    appsink_ = GST_APP_SINK(tmpsink);
    return true;
}

cv::Size2i GstCVCamera::getWidthHeightFromCaps(GstCaps *gstCaps)
{
    cv::Size2i retVal(0,0);

    if (!gstCaps)
    {
        qDebug() << "gstreamer capabilities are null";
        return retVal;
    }

    const GstStructure* gstCapsStruct = gst_caps_get_structure(gstCaps, 0);
    if (!gstCapsStruct) {
        qDebug() << "gstreamer capabilities has no structure";
        return retVal;
    }

    if (!gst_structure_get_int(gstCapsStruct, "width", &retVal.width)) {
        qDebug() << ("gstreamer capabilities has no width\n");
        return retVal;
    }

    if (!gst_structure_get_int(gstCapsStruct, "height", &retVal.height)) {
        qDebug() << ("gstreamer capabilities has no height\n");
        return retVal;
    }

    if (retVal.width < 1 || retVal.height < 1) {
        qDebug() << "gst caps width: " + QString::number(retVal.width) +
                    " or height: " + QString::number(retVal.height) + " invalid";
        return retVal;
    }

    return retVal;
}

bool GstCVCamera::changePipelineState(const GstState &state) {
    GstStateChangeReturn retVal = gst_element_set_state(pipeline_, state);

    if (retVal == GST_STATE_CHANGE_FAILURE) {
        gst_object_unref(pipeline_);
        qDebug() << ": unable to set the GStreamer pipeline to the state " + QString(gst_element_state_get_name(state));
        return false;
    }

    retVal = gst_element_get_state(pipeline_, nullptr, nullptr, GST_SECOND * 2);

    if (retVal == GST_STATE_CHANGE_FAILURE) {
        qDebug() << ": GStreamer pipeline state change failure";
        return false;
    }

    return true;
}

cv::Mat GstCVCamera::preprocessData(GstBuffer *, const GstMapInfo &mappedBuffer, GstCaps *bufferCaps) {
    const auto wh = getWidthHeightFromCaps(bufferCaps);

    if (wh.width == -1 || wh.height == -1) {
        qDebug() << QString::fromStdString( "Failed to obtain width and height from caps of the appsink");
    }

    // Make dummy with data pointer -> it is not copied. We need it to check the bufferSize against the data size
    cv::Mat dummy(wh.height, wh.width, CV_8UC1, static_cast<void*>(mappedBuffer.data));

    if ((dummy.elemSize() * dummy.total()) != mappedBuffer.size) {
        qDebug() << QString::fromStdString( "Expected size of the acquired image mismatch the buffersize");
        return cv::Mat();
    }

    return dummy.clone();
}

}

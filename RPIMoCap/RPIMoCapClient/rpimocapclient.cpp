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

#include "rpimocapclient.h"



RPIMoCapClient::RPIMoCapClient(QObject *parent)
    : QObject(parent)
    , m_camera("v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=90/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink max-buffers=1 name=appsink") {

}

void RPIMoCapClient::onLines(const std::vector<RPIMoCap::Line3D> &lines)
{
    std::stringstream buf;
    msgpack::pack(buf, lines);
    emit linesSerialized(QByteArray::fromStdString(buf.str()));
}

void RPIMoCapClient::trigger()
{
    if (!opened) {
        m_camera.open();
        opened = true;
    }

    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat currentImage = m_camera.pullData();
    //cv::imwrite("/tmp/image.jpg",currentImage);

    if (currentImage.empty())
    {
        qDebug() << "empty image from gst";
    }

    onLines(m_markerDetector.onImage(currentImage));

    auto end = std::chrono::high_resolution_clock::now();

    qDebug() << "elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

}

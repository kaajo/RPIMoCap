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

#ifndef POINTCHECKER_H
#define POINTCHECKER_H


#include <RPIMoCap/Core/frame.h>
#include <RPIMoCap/Core/line3d.h>
#include <RPIMoCap/Core/msgpack_defs.h>

//#include "munkres.h"

#include <QVector>
#include <QQueue>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <vector>

enum class PointCount
{
    TOOMANY,
    GOOD,
    NOTENOUGH,
    NO
};

class PointChecker
{
    PointCount state = PointCount::NO;

    size_t noFrameDuration = 0;
    size_t maxNoFrameDuration = 30;
    size_t wrongFrameDuration = 0;
    size_t maxWrongFrameDuration; ///not using
    size_t maxIndex = 0;
    int numOfPoints = 1;

    QQueue<size_t> lastRemovedIDs;

    QVector<RPIMoCap::Frame::Marker> lastPoints;
    QVector<RPIMoCap::Frame::Marker> lastGoodFrame;

public:
    PointChecker();

    size_t getNumOfPoints() const;
    void setNumOfPoints(const size_t &value);

    QVector<RPIMoCap::Frame::Marker> getLastPoints() const {return lastPoints;}

    QVector<RPIMoCap::Frame::Marker> solvePointIDs(QVector<Eigen::Vector3f> points2);
private:

    QVector<RPIMoCap::Frame::Marker> handleNo(QVector<Eigen::Vector3f> &points);
    QVector<RPIMoCap::Frame::Marker> handleNotEnough(QVector<Eigen::Vector3f> &points);
    QVector<RPIMoCap::Frame::Marker> handleGood(QVector<Eigen::Vector3f> &points);

    std::vector<std::vector<double> > createDistanceMap(QVector<RPIMoCap::Frame::Marker> lastPoints, QVector<Eigen::Vector3f> points);
    void checkRemovedIndexes();
    size_t nextUniqueIndex(int size);
    void addUncoveredPoints(QVector<Eigen::Vector3f> points, std::vector<std::vector<double> > map, QVector<RPIMoCap::Frame::Marker> &pts);
    QVector<RPIMoCap::Frame::Marker> addCoveredPoints(QVector<Eigen::Vector3f> points, std::vector<std::vector<double> > map);
    void handleRemovedPoints(QVector<RPIMoCap::Frame::Marker> points);

    void my_solve(std::vector <std::vector <double> > &m)
    {
      // Matrix<double> matrix = convert_std_2d_vector_to_munkres_matrix<double>(m);
      // Munkres munkres;
      // munkres.solve (matrix);
      // fill_std_2d_vector_from_munkres_matrix<double>(m, matrix);
    }
};

#endif // POINTCHECKER_H

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

#include "Server/pointchecker.h"

#include <QVector>
#include <QVector2D>

size_t PointChecker::getNumOfPoints() const
{
    return numOfPoints;
}

void PointChecker::setNumOfPoints(const size_t &value)
{
    numOfPoints = value;
}

PointChecker::PointChecker()
{
}

QVector<RPIMoCap::Frame::Marker> PointChecker::solvePointIDs(QVector<Eigen::Vector3f> points)
{
    QVector<RPIMoCap::Frame::Marker> pts;

    if(points.empty())
    {
        state = PointCount::NO;
        ++noFrameDuration;
        return pts;
    }

    switch (state) {
    case PointCount::NO:
    {
        pts = handleNo(points);
        break;
    }
    case PointCount::NOTENOUGH:
    {
        pts = handleNotEnough(points);
        break;
    }
    case PointCount::GOOD:
    {
        pts = handleGood(points);
        break;
    }
    case PointCount::TOOMANY:
    {
        pts = handleNotEnough(points);
        break;
    }
    default:
        break;
    }

    if(points.size() < lastPoints.size())
    {
        handleRemovedPoints(pts);
    }

    noFrameDuration = 0;
    lastPoints = pts;

    return pts;

}

QVector<RPIMoCap::Frame::Marker> PointChecker::handleNo(QVector<Eigen::Vector3f> &points)
{
    QVector<RPIMoCap::Frame::Marker> pts;

    if(points.size() < numOfPoints)
    {
        state = PointCount::NOTENOUGH;
    }
    else if(points.size() == numOfPoints)
    {
        state = PointCount::GOOD;
    }
    else if(points.size() > numOfPoints)
    {
        state = PointCount::TOOMANY;
    }

    if(noFrameDuration < maxNoFrameDuration && !lastGoodFrame.empty())
    {
        auto m = createDistanceMap(lastGoodFrame, points);

        if(! m.empty())
        {
            my_solve(m);
        }

        pts = addCoveredPoints(points, m);
    }
    else
    {
        lastRemovedIDs.clear();

        for(int i = 0; i < points.size(); i++)
        {
            pts.push_back({nextUniqueIndex(i), points[i]});
        }
    }

    return pts;
}

QVector<RPIMoCap::Frame::Marker> PointChecker::handleNotEnough(QVector<Eigen::Vector3f> &points)
{
    QVector<RPIMoCap::Frame::Marker> pts;

    std::vector< std::vector<double>> m = createDistanceMap(lastPoints, points);

    if(! m.empty())
    {
        my_solve(m);
    }

    pts = addCoveredPoints(points,m);

    if(points.size() < numOfPoints)
    {
        state = PointCount::NOTENOUGH;

        addUncoveredPoints(points,m,pts);

    }
    else if(points.size() == numOfPoints)
    {
        state = PointCount::GOOD;

        if(!lastGoodFrame.empty())
        {
            m = createDistanceMap(lastGoodFrame, points);

            if(! m.empty())
            {
                my_solve(m);
            }
        }
        addUncoveredPoints(points, m, pts);

    }
    else if(points.size() > numOfPoints)
    {
        state = PointCount::TOOMANY;

        addUncoveredPoints(points, m, pts);
    }

    return pts;
}

QVector<RPIMoCap::Frame::Marker> PointChecker::handleGood(QVector<Eigen::Vector3f> &points)
{
    QVector<RPIMoCap::Frame::Marker> pts;

    std::vector< std::vector<double>> m = createDistanceMap(lastPoints, points);

    if(! m.empty())
    {
        my_solve(m);
    }

    pts = addCoveredPoints(points,m);

    if(pts.size() == numOfPoints)
    {
        state = PointCount::GOOD;
        lastGoodFrame = pts;
    }
    else
    {
        state = PointCount::NOTENOUGH;
    }

    if(points.size() > numOfPoints)
    {
        for(int i = 0; i < points.size(); i++)
        {
            bool ok = false;
            for(int j = 0; j < pts.size(); j++)
            {
                Eigen::Vector3f pnt = pts[j].position;

                if(points[i] == pnt)
                {
                    ok = true;
                    break;
                }
            }
            if(!ok)
            {
                points.erase(points.begin()+i);
            }
        }
    }

    return pts;
}


std::vector<std::vector<double> > PointChecker::createDistanceMap(QVector<RPIMoCap::Frame::Marker> lastPoints, QVector<Eigen::Vector3f> points)
{
    std::vector< std::vector<double>> matrix;

    for(int i = 0; i < lastPoints.size(); i++)
    {
        std::vector<double> vec;
        for(int j = 0; j < points.size(); j++)
        {
            Eigen::Vector3f f = lastPoints[i].position;

            vec.push_back((f - points[j]).norm());
        }
        matrix.push_back(vec);
    }

return matrix;
}

size_t PointChecker::nextUniqueIndex(int size)
{
    if(lastRemovedIDs.empty())
    {
        return size;
    }
    else
    {
        size_t val = lastRemovedIDs.front();
        lastRemovedIDs.pop_front();
        return val;
    }
}

void PointChecker::addUncoveredPoints(QVector<Eigen::Vector3f> points, std::vector<std::vector<double> > map, QVector<RPIMoCap::Frame::Marker> &pts)
{
    size_t rows = map.size();

    if(rows == 0)
    {
        return;
    }

    size_t cols = map[0].size();

    for(size_t i = 0; i < cols; i++)
    {
        bool ok  = false;
        for(size_t j = 0; j < rows; j++)
        {
            if(map[j][i] == 0)
            {
                ok = true;
                break;
            }
        }
        if(!ok)
        {
            pts.push_back({nextUniqueIndex(pts.size()), points[i]});
        }
    }
}

QVector<RPIMoCap::Frame::Marker> PointChecker::addCoveredPoints(QVector<Eigen::Vector3f> points, std::vector<std::vector<double> > map)
{
    QVector<RPIMoCap::Frame::Marker> pts;

    for(size_t i = 0; i < map.size(); i++)
    {
        for(size_t j = 0; j < map[i].size(); j++)
        {
            if(map[i][j] == 0)
            {
                if(i < lastPoints.size() && j < points.size())
                {
                    pts.push_back({lastPoints[i].id, points[j]});
                }
            }
        }
    }

    return pts;
}

void PointChecker::handleRemovedPoints(QVector<RPIMoCap::Frame::Marker> points)
{
    for(int i = 0; i < lastPoints.size(); i++)
    {
        bool found = false;
        for(int j = 0; j < points.size(); j++)
        {
            if(lastPoints[i].id == points[j].id)
            {
                found = true;
                break;
            }
        }
        if(!found)
        {
            if(lastPoints[i].id <= lastPoints.size())
                lastRemovedIDs.push_back(lastPoints[i].id);
        }
    }
}

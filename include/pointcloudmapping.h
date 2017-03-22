/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <condition_variable>
#include <pcl/visualization/cloud_viewer.h>

using namespace ORB_SLAM2;

class IMAGE;

class PointCloudMapping
{
public:
    typedef pcl::visualization::CloudViewer::ColorCloud myPointCloud;

    PointCloudMapping(double resolution_, Map* mpMap);

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* pKF, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();

protected:
    myPointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    shared_ptr<thread>  viewerThread;

    bool    shutDownFlag    =false;
    mutex   shutDownMutex;

    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;

    // rgb and depth data to generate point clouds
    map<long unsigned int,cv::Ptr<IMAGE>> kfmap;

    mutex                   keyframeMutex;

    double resolution = 0.04;
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB>  voxel;

    Map* pMap;
public:
    myPointCloud::Ptr globalMap;
    mutex			globalMutex;
};

#endif // POINTCLOUDMAPPING_H

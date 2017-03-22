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

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

typedef pcl::visualization::CloudViewer::ColorCloud myPointCloud;

PointCloudMapping::PointCloudMapping(double resolution_, Map* mpMap ):pMap(mpMap)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared<myPointCloud>();

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* pKF, cv::Mat& color, cv::Mat& depth)
{
    //cout<<"receive a keyframe, id = "<<pKF->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);

    cv::Ptr<IMAGE> ptr(new IMAGE(color.clone(),depth.clone()));
    kfmap.insert(map<long unsigned int,cv::Ptr<IMAGE>>::value_type(pKF->mnId,ptr));

    keyFrameUpdated.notify_one();
}

myPointCloud::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    //cout << "depth = "<< endl << " " << depth << endl << endl;
    myPointCloud::Ptr tmp( new myPointCloud() );
    // point cloud is null ptr

    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {

            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;

            pcl::PointXYZRGB p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    myPointCloud::Ptr cloud(new myPointCloud());
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    //cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* ptr_void)
{
    PointCloudMapping* ptr=(PointCloudMapping*)ptr_void;
    if (event.getKeySym() == "s" && event.keyDown())
    {
        unique_lock<mutex> lck(ptr->globalMutex);
        boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
        pcl::io::savePCDFile( "../pointcloud/pointcloud_"+boost::posix_time::to_simple_string(now)+".pcd", *(ptr->globalMap) );
    }
}


void PointCloudMapping::viewer()
{
    boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer("3D Viewer"));
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        vector<KeyFrame*> vpKFs;
        {
            unique_lock<mutex> lck( keyframeMutex );
            vpKFs = pMap->GetAllKeyFrames();
        }
        {
        	unique_lock<mutex> lck(globalMutex);
		    globalMap->clear();
		    for ( size_t i=0; i<vpKFs.size() ; i++ )
		    {
		        if(vpKFs[i]->mnId == 0)
		            continue;
		        myPointCloud::Ptr p = generatePointCloud( vpKFs[i], kfmap[vpKFs[i]->mnId]->color, kfmap[vpKFs[i]->mnId]->depth );
		        *globalMap += *p;
		    }

		    myPointCloud::Ptr tmp(new myPointCloud());
		    voxel.setInputCloud( globalMap );
		    voxel.filter( *tmp );
		    globalMap->swap( *tmp );
        }
        viewer->registerKeyboardCallback(keyboardEventOccurred,(void*)this);
        viewer->showCloud( globalMap );
        //cout << "show global map, size=" << globalMap->points.size() << endl;
    }
}
/*TODO
统计滤波！！
*/
/*
pcl::filters::GaussianKernelRGB<pcl::PointXYZRGB,pcl::PointXYZRGB> sor_in;
sor_in.setInputCloud(globalMap);
sor_in.setSigma(0.1);
sor_in.setThreshold(1.0);
sor_in.setThresholdRelativeToSigma(0.1f);
sor_in.initCompute();
sor_in.filter(cloud_out_filtered);
*/

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
 * along with this program.  If not, see <http://www.gnu.org/licenses/ >.
 *
 * 添加一个 PointCloudMapping 类并创建一个线程对 关键帧 的点云信息进行处理和显示
 */

#include "PointCloudMapping.h"
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

namespace ORB_SLAM2 {

/**
 * @brief: 构造函数
 * @param [double] resolution_:
 * @param [double] meank_:
 * @param [double] thresh_:
 * @return [*]
 */
PointCloudMapping::PointCloudMapping(double resolution_, double meank_,
                                     double thresh_) {
  //  统计滤波器设置
  this->resolution = resolution_;
  this->meank = meank_;
  this->thresh = thresh_;
  // 在对象构造的时候，已经对统计滤波和体素滤波进行了设置，所以后面直接使用即可
  //  对每个点分析的临近点的个数设置为50
  statistical_filter.setMeanK(meank);
  //  标准差的倍数设置为 th=1
  statistical_filter.setStddevMulThresh(thresh);

  //  体素滤波设置
  voxel.setLeafSize(resolution, resolution, resolution);
  //  全局点云地图
  globalMap = boost::make_shared<PointCloud>();

  // TODO: 绑定并启动显示线程, 使用Viewer()函数
  // std::bind bind绑定类成员函数时，第一个参数表示对象的成员函数的指针，第二个参数表示对象的地址。
  viewerThread = make_shared<thread>(std::bind(&PointCloudMapping::Viewer, this));
  //  标记参数
  loopbusy = false;
  cloudbusy = false;
}

void PointCloudMapping::Shutdown() {
  {
    unique_lock<mutex> lck(shutDownMutex);
    shutDownFlag = true;
  }
  viewerThread->join();
  keyFrameUpdated.notify_one();
}

/**
 * @brief 将关键帧生成点云并插入到全局点云中
 *
 * @param kf            关键帧
 * @param color         关键帧对应的彩色图
 * @param depth         关键帧对应的深度图
 * @param idk           关键帧索引，不管是否增加关键帧，都计数+1
 */
void PointCloudMapping::InsertKeyFrame(KeyFrame *kf, cv::Mat &color,
                                       cv::Mat &depth, int idk) {
  cout << "receive a keyframe, id = " << idk << " 第" << kf->mnId << "个"
       << endl;
  unique_lock<mutex> lck(keyframeMutex);

  // 当前关键帧
  keyframes.push_back(kf);

  //  获取本帧图像关联的点云id, 相对于世界坐标系的变换Twc
  PointCloude *pointcloude = new PointCloude;
  //  记录关键帧索引
  (*pointcloude).pcID = idk;
  //  将关键帧位姿Tcw转化为符合PCL格式
  (*pointcloude).T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
  //  联合深度图和RGB图像，找到对应点的空间位置，并且构造空间点云．
  (*pointcloude).pcE = GeneratePointCloud(kf, color, depth);

  {
    //  存储所有关键帧的点云、位姿、id信息到pointcloud
    unique_lock<mutex> lck(pointcloudMutex);
    pointcloud.push_back((*pointcloude));
  }

  //  点云更新通知。notify_one唤醒某个等待(wait)线程。如果当前没有等待线程，则该函数什么也不做
  keyFrameUpdated.notify_one();
}


// 我们使用OpenCV的imread函数读取图片。在OpenCV2里，图像是以矩阵(cv::MAt)作为基本的数据结构。Mat结构既可以帮你管理内存、像素信息，还支持一些常见的矩阵运算，是非常方便的结构。彩色图像含有R,G,B三个通道，每个通道占8个bit（也就是unsigned char），故称为8UC3（8位unsigend char, 3通道）结构。而深度图则是单通道的图像，每个像素由16个bit组成（也就是C++里的unsigned short），
// 像素的值代表该点离传感器的距离。通常1000的值代表1米，所以我们把camera_factor设置成1000. 这样，深度图里每个像素点的读数除以1000，就是它离你的真实距离了。
pcl::PointCloud<PointCloudMapping::PointT>::Ptr
PointCloudMapping::GeneratePointCloud(KeyFrame *kf, cv::Mat &color,
                                      cv::Mat &depth) {
  // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
  PointCloud::Ptr tmp(new PointCloud());
  for (int m = 0; m < depth.rows; m += 3) {
    for (int n = 0; n < depth.cols; n += 3) {
      // TODO: 反投影生成点云
      // 提示： 注意点云有效范围
      // 取出m行n列元素的值
      float d = depth.ptr<float>(m)[n];
      // 筛选点云
      if (d > 5 || d < 0.01)  
        continue;
      // pcl::PointXYZRGBA
      PointT point;
      point.z = d;
      point.x = (n - kf->cx) * d / kf->fx;
      point.y = (m - kf->cy) * d / kf->fy;
      // opencv读取图片格式是BGR
      point.b = color.ptr<uchar>(m)[n * 3];
      point.g = color.ptr<uchar>(m)[n * 3 + 1];
      point.r = color.ptr<uchar>(m)[n * 3 + 2];
      
      tmp->points.push_back(point);

    }
  }

  if (tmp->points.size()) {
    cout << "点云数目：" << tmp->points.size() << endl;
  } else {
    cout << "点云为空！" << endl;
  }
  return tmp;
}

void PointCloudMapping::Viewer() {
  pcl::visualization::CloudViewer viewer("viewer");
  while (1) {
    {
      unique_lock<mutex> lck_shutdown(shutDownMutex);
      if (shutDownFlag) {
        cout << "Shutdown PointCloud Viewer!" << endl;
        break;
      }
    }

    {
      //  等待关键帧的更新
      unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
      keyFrameUpdated.wait(lck_keyframeUpdated);
    }

    size_t N = 0;
    {
      unique_lock<mutex> lck(keyframeMutex);
      N = keyframes.size();
      cout << "keyframes to be processed:" << N << endl;
    }

    {
      unique_lock<mutex> lck(loopBusyMutex);
      if (loopbusy) continue;
    }

    {
      unique_lock<mutex> lck(cloudBusyMutex);
      // 没有新的关键帧插入
      if (lastKeyframeSize == N) cloudbusy = false;
      cloudbusy = true;
    }

    // 遍历关键帧序列的每一帧，提取出每一帧中的点云，并且变换到世界坐标系下,最后加入到全局地图中；
    {
      // 锁住全局地图， 准备更新
      unique_lock<mutex> lck(globalMapMutex);
      {
        // 锁住待处理的关键帧队列和点云队列
        unique_lock<mutex> lck(keyframeMutex), lck2(pointcloudMutex);

        for (size_t i = lastKeyframeSize; i < N; i++) {
          PointCloud::Ptr p(new PointCloud);
          // TODO: 将每一个关键帧中的点云通过刚体变换变换到世界坐标系下；
          // 提示：pcl::transformPointCloud();
          // pcl::transformPointCloud(*source_cloud, *target_cloud, transform)
          // 其中source_cloud, target_cloud的类型为pcl::PointCloud<pcl::PointXYZ>::Ptr
          // 函数的作用是通过转换矩阵transform将source_cloud转换后存到target_cloud中保存。
          // 第一个变量，相机坐标系下的点云， 第二个变量，我们自己创建的*p，因为之后要加入globalMap里面，第三个，位姿变换
          // 在定义的数据结构中，是Tcw, Twc求逆, matrix()返回变换对应的矩阵，T输出时用此函数。
          pcl::transformPointCloud(*(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
          //  将变换后的点云加入到全局地图中；
          *globalMap += *p;
        }
      }

      PointCloud::Ptr tmp(new PointCloud);
      PointCloud::Ptr tmp1(new PointCloud);

      // TODO: 全局点云作为输入，通过统计滤波器
      // 统计滤波的基本使用流程
      // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      // sor.setInputCloud (cloud);
      // sor.setMeanK (50);  //计算平均距离时，设置邻域点数量
      // sor.setStddevMulThresh (1.0);//设置标准差阈值倍数
      // sor.filter (*cloud_filtered);
      // 过滤后的结果以索引数组存储
      statistical_filter.setInputCloud(globalMap);
      statistical_filter.filter(*tmp1);
      // TODO:
      // 统计滤波的输出作为体素滤波器的输入，通过滤波的点输出到全局地图中；
      voxel.setInputCloud(tmp1);
      voxel.filter(*tmp);
      // pcl::swap用于交换两个点云
      // 经过两次滤波的点云进入globalmap
      globalMap->swap(*tmp);
      viewer.showCloud(globalMap);
      cout << "viewer() :   show global map, size=" << N << "   "
           << globalMap->points.size() << endl;
    }  // 释放全局地图

    {
      unique_lock<mutex> lck(cloudBusyMutex);
      cloudbusy = false;
    }

    lastKeyframeSize = N;
  }
}

void PointCloudMapping::Save() {
  pcl::io::savePCDFile("result_1.pcd", *globalMap);
  cout << "globalMap save finished" << endl;
}

//  闭环线程中GBA后用于更新点云
void PointCloudMapping::UpdateCloud(vector<KeyFrame *> all_kfs) {
  {
    unique_lock<mutex> lck(cloudBusyMutex);
    // 点云不忙，没在更新才更新因为loop变化的电晕
    if (!cloudbusy) {
      {
        unique_lock<mutex> lck{loopBusyMutex};
        loopbusy = true;
      }
      // 提取每一个关键帧，提取每一帧上面的点云，用GBA后的位姿转换到世界坐标系下
      //  新建临时点云tmp1
      PointCloud::Ptr tmp1(new PointCloud);

      {
        // 锁住点云队列
        unique_lock<mutex> lck2(pointcloudMutex);
        //  遍历所有关键帧
        for (int i = 0; i < all_kfs.size(); i++) {
          //  pointcloud存储的是每个关键帧的点云、位姿、id信息
          for (int j = 0; j < pointcloud.size(); j++) {
            // TODO:
            // 找到具有相同的id的关键帧对应的点云，用GBA后的位姿来重新更新点云
            if (all_kfs[i]->mnFrameId == pointcloud[j].pcID) {
              // 说明当前点云能被该关键帧观测到，该关键帧的位姿发生了更新，因此我们也更新一下点云位置
              // 此处隐藏cv::Mat到eigen的转换,利用ORB2的convert
              Eigen::Isometry3d Tcw_cur = ORB_SLAM2::Converter::toSE3Quat(all_kfs[i]->GetPose());
              PointCloud::Ptr Cloud_ptr(new PointCloud);
              pcl::transformPointCloud(*(pointcloud[j].pcE), *Cloud_ptr, Tcw_cur.inverse().matrix());
              *tmp1 += *Cloud_ptr; // 更新至临时点云
              continue;  
            }
          }
        }
      }

      cout << "finish adjusting all the point cloud!" << endl;

      // TODO: 点云比较稠密，体素滤波来降采tmp1样
      // 提示： tmp1->体素滤波-> tmp2
      PointCloud::Ptr tmp2(new PointCloud());  //  tmp2存储滤波后的点云
      voxel.setInputCloud(tmp1);
      voxel.filter(*tmp2);
      {
        unique_lock<mutex> lck(globalMapMutex);
        // TODO: 替换全局点云   
        // 我们创建了临时点云tmp1 并利用GBA更新，然后利用体素滤波过滤，现在我们要将其加入最终的地图了。
        globalMap->swap(*tmp2);
      }

      {
        unique_lock<mutex> lck{loopBusyMutex};
        loopbusy = false;
      }
    }
    cout << "after cloudupdate, loopbusy:" << loopbusy <<endl;
  }
}

}  // namespace ORB_SLAM2

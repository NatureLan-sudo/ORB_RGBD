/*
 * @Author: NatureLan-sudo lantianran282@163.com
 * @Date: 2023-02-25 23:27:08
 * @LastEditors: NatureLan-sudo lantianran282@163.com
 * @LastEditTime: 2023-02-26 21:05:46
 * @FilePath: /final_test/include/PointCloude.h
 * @brief: 
 * 
 * Copyright (c) 2023 by Nature, All Rights Reserved. 
 */
#ifndef POINTCLOUDE_H
#define POINTCLOUDE_H
/** * * ━━━━━━神兽出没━━━━━━
 * 　　　┏┓　　　┏┓
 * 　　┃　　　　　　　┃
 * 　　┃　　　━　　　┃
 * 　　┃　┳┛　┗┳　┃
 * 　　┃　　　　　　　┃
 * 　　┃　　　┻　　　┃
 * 　　┃　　　　　　　┃
 * 　　┗━┓　　　┏━┛Code is far away from bug with the animal rotecting
 * 　　　　┃　　　┃ 神兽保佑,代码无bug
 * 　　　　┃　　　┃
 * 　　　　┃　　　┗━━━┓
 * 　　　　┃　　　　　　　┣┓
 * 　　　　┃　　　　　　　┏┛
 * 　　　　┗┓┓┏━┳┓┏┛
 * 　　　　　┃┫┫　┃┫┫
 * 　　　　　┗┻┛　┗┻┛
 * * ━━━━━━感觉萌萌哒━━━━━━ */ /** 
* 　　　　　　　　┏┓　　　┏┓ 
* 　　　　　　　┏┛┻━━━┛┻┓ 
* 　　　　　　　┃　　　　　　 ┃ 　 
* 　　　　　　　┃　　　━　　 ┃ 
* 　　　　　　　┃　＞ ＜  ┃ 
* 　　　　　　　┃　　　　　　 ┃ 
* 　　　　　　　┃...　⌒　.┃ 
* 　　　　　　　┃　　　　　　　┃ 
* 　　　　　　　┗━┓　　　┏━┛ 
* 　　　　　　　　　┃　　　┃　Code is far away from bug with the animal protecting　　　　　　　　　　 
* 　　　　　　　　　┃　　　┃ 神兽保佑,代码无bug 
* 　　　　　　　　　┃　　　┃　　　　　　　　　　　 
* 　　　　　　　　　┃　　　┃ 　　　　　　 
* 　　　　　　　　　┃　　　┃ 
* 　　　　　　　　　┃　　　┃　　　　　　　　　　　 
* 　　　　　　　　　┃　　　┗━━━┓ 
* 　　　　　　　　　┃　　　　　　　┣┓ 
* 　　　　　　　　　┃　　　　　　　┏┛ 
* 　　　　　　　　　┗┓┓┏━┳┓┏┛ 
* 　　　　　　　　　　┃┫┫　┃┫┫ 
* 　　　　　　　　　　┗┻┛　┗┻┛ 
*
/ /** 

*　　　　　　　　┏┓　　　┏┓+ + 
*　　　　　　　┏┛┻━━━┛┻┓ + + 
*　　　　　　　┃　　　　　　　┃ 　 
*　　　　　　　┃　　　━　　　┃ ++ + + + 
*　　　　　　 ████━████ ┃+ 
*　　　　　　　┃　　　　　　　┃ + 
*　　　　　　　┃　　　┻　　　┃ 
*　　　　　　　┃　　　　　　　┃ + + 
*　　　　　　　┗━┓　　　┏━┛ 
*　　　　　　　　　┃　　　┃　　　　　　　　　　　 
*　　　　　　　　　┃　　　┃ + + + + 
*　　　　　　　　　┃　　　┃　　　　Code is far away from bug with the animal protecting　　　　　　　 
*　　　　　　　　　┃　　　┃ + 　　　　神兽保佑,代码无bug　　 
*　　　　　　　　　┃　　　┃ zitai
*　　　　　　　　　┃ 　　　　　　　┣┓ 
*　　　　　　　　　┃ 　　　　　　　┏┛ 
*　　　　　　　　　┗┓┓┏━┳┓┏┛ + + + + 
*　　　　　　　　　　┃┫┫　┃┫┫ 
*　　　　　　　　　　┗┻┛　┗┻┛+ + + + 
*/
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <mutex>

namespace ORB_SLAM2 {

// 自定义的扩展点云类型，包括：点云、位姿、id
class PointCloude {
  typedef pcl::PointXYZRGBA PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

 public:
  PointCloud::Ptr pcE; // 点云坐标

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Isometry3d T; // 姿态
  int pcID;
};

}  // namespace ORB_SLAM2

#endif  // POINTCLOUDE_H

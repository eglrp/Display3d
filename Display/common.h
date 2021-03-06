/*
 * Created by 郭嘉丞 on 15/9/12.
 * Last edited by sugar10w, 2016.5.10
 *
 * 为最常用的几个点云库类型提供简写。
 * 定义一些编译宏开关。
 *
 */

#ifndef KINECTDATAANALYZER_COMMON_H
#define KINECTDATAANALYZER_COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* 点 */
typedef pcl::PointXYZRGB PointT;
/* 点云 */
typedef pcl::PointCloud<PointT> PointCloud;
/* 点云指针 */
typedef PointCloud::Ptr PointCloudPtr;
/* 点云常量指针 */
typedef PointCloud::ConstPtr PointCloudConstPtr;

/* 法向量点 */
typedef pcl::Normal PointNT;
/* 法向量点云 */
typedef pcl::PointCloud<PointNT> PointCloudNT;
/* 法向量点云指针 */
typedef PointCloudNT::Ptr PointCloudNTPtr;
/* 法向量点云常量指针 */
typedef PointCloudNT::ConstPtr PointCloudNTConstPtr;

#endif //KINECTDATAANALYZER_COMMON_H

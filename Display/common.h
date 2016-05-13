/*
 * Created by ����ة on 15/9/12.
 * Last edited by sugar10w, 2016.5.10
 *
 * Ϊ��õļ������ƿ������ṩ��д��
 * ����һЩ����꿪�ء�
 *
 */

#ifndef KINECTDATAANALYZER_COMMON_H
#define KINECTDATAANALYZER_COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* �� */
typedef pcl::PointXYZRGB PointT;
/* ���� */
typedef pcl::PointCloud<PointT> PointCloud;
/* ����ָ�� */
typedef PointCloud::Ptr PointCloudPtr;
/* ���Ƴ���ָ�� */
typedef PointCloud::ConstPtr PointCloudConstPtr;

/* �������� */
typedef pcl::Normal PointNT;
/* ���������� */
typedef pcl::PointCloud<PointNT> PointCloudNT;
/* ����������ָ�� */
typedef PointCloudNT::Ptr PointCloudNTPtr;
/* ���������Ƴ���ָ�� */
typedef PointCloudNT::ConstPtr PointCloudNTConstPtr;

#endif //KINECTDATAANALYZER_COMMON_H

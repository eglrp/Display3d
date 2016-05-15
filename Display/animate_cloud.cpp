/* Created by sugar10w, 2016.5.10
 * Last edited by sugar10w, 2016.5.13
 * 
 * 可以动画的点云
 */

#include "animate_cloud.h"

AnimateCloud::AnimateCloud(PointCloudPtr cloud, std::string reg_name)
    : target_cloud_(cloud), display_cloud_(new PointCloud), plane_cloud_(new PointCloud),
    status_(AnimateStatus::Stop), 
    size_(cloud->points.size()),
    count_(0), max_count_(0),
    visible_(false),
    reg_name_(reg_name),
    k0(1.4), k1(0.5), k2(0.5)
{
    // copy to display_cloud_
    display_cloud_->resize(size_);
    for (int i=0; i<size_; ++i) {
        PointT & p1 = display_cloud_->points[i];
        PointT & p2 = target_cloud_->points[i];
        p1.x = p1.y = p1.z = 0;
        p1.r = p2.r; p1.g = p2.g; p1.b = p2.b;
    }
}

AnimateCloud::~AnimateCloud()
{
}

void AnimateCloud::setAnimateStatus(AnimateStatus status)
{
    status_ = status; 

    switch (status_) {
    case AnimateStatus::Stop:
        count_ = 0; max_count_ = 0;
        break;
    case AnimateStatus::Appear:
        for (int i=0; i<size_; ++i) {
            PointT & pt1 = display_cloud_->points[i];
            PointT & pt2 = target_cloud_->points[i];
            pt1.x = pt2.x;
            pt1.y = pt2.y;
            pt1.z = pt2.z;
        }
        visible_ = true;
        break;
    case AnimateStatus::Disappear:
        for (int i=0; i<size_; ++i) {
            PointT & pt1 = display_cloud_->points[i];
            pt1.x =  pt1.y = pt1.z = 0;
        }
        visible_ = false; 
        break;
    case AnimateStatus::Target:
        visible_ = true; 
        count_ = 0; max_count_ = 30; 
        break;
    case AnimateStatus::Plane:
        visible_ = true; 
        count_ = 0; max_count_ = 30; 
        if (plane_cloud_->points.size()==0) {
            // calculate plane_cloud_
            plane_cloud_->resize(size_);
            for (int i=0; i<size_; ++i) {
                PointT & pt1 = plane_cloud_->points[i];
                PointT & pt2 = target_cloud_->points[i];
                if (pt2.z!=0) {
                    pt1.x = pt2.x / pt2.z;
                    pt1.y = pt2.y / pt2.z;
                    pt1.z = 1;
                }
                else {
                    pt1.x = pt1.y = pt1.z = 0;
                }
            }
        }
        break;
    case AnimateStatus::Counting:
        visible_ = true; 
        count_ = 0; max_count_ = 60; 
        break;
    case AnimateStatus::Bursting:
        if (visible_) { count_ = 0; max_count_ = 5; }
        break;
    case AnimateStatus::Zero:
        if (visible_) { count_ = 0; max_count_ = 20; }
        break;
    }
}

void AnimateCloud::Refresh()
{
    switch(status_)
    {
    case AnimateStatus::Stop:
        break;
    case AnimateStatus::Target:
        for (int i=0; i<size_; ++i) {
            PointT & p1 = display_cloud_->points[i];
            PointT & p2 = target_cloud_->points[i];
            p1.x = p1.x * k1 + p2.x * k2;
            p1.y = p1.y * k1 + p2.y * k2;
            p1.z = p1.z * k1 + p2.z * k2;
        }
        ++count_; if (count_==max_count_) setAnimateStatus(AnimateStatus::Stop);
        break;
    case AnimateStatus::Plane:
        for (int i=0; i<size_; ++i) {
            PointT & p1 = display_cloud_->points[i];
            PointT & p2 = plane_cloud_->points[i];
            p1.x = p1.x * k1 + p2.x * k2;
            p1.y = p1.y * k1 + p2.y * k2;
            p1.z = p1.z * k1 + p2.z * k2;
        }
        ++count_; if (count_==max_count_) setAnimateStatus(AnimateStatus::Stop);
        break;
    case AnimateStatus::Counting:
        for (int i=size_/max_count_*count_;
                 i<size_/max_count_*(count_+1); ++i) {
            if (i>=size_) break;
            PointT & p1 = display_cloud_->points[i];
            PointT & p2 = target_cloud_->points[i];
            p1.x = p2.x;
            p1.y = p2.y;
            p1.z = p2.z;
        }
        ++count_; if (count_==max_count_) setAnimateStatus(AnimateStatus::Stop);
        break;
    case AnimateStatus::Bursting:
        for (int i=0; i<size_; ++i) {
            PointT & p1 = display_cloud_->points[i];
            PointT & p2 = plane_cloud_->points[i];
            p1.x = p1.x * k0;
            p1.y = p1.y * k0;
            p1.z = p1.z * k0;
        }
        ++count_; if (count_==max_count_) { setAnimateStatus(AnimateStatus::Stop); visible_ = false; }
        break;
    case AnimateStatus::Zero:
        for (int i=0; i<size_; ++i) {
            PointT & p1 = display_cloud_->points[i];
            p1.x = p1.x * k1;
            p1.y = p1.y * k1;
            p1.z = p1.z * k1;
        }
        ++count_; if (count_==max_count_) { setAnimateStatus(AnimateStatus::Stop); visible_ = false; }
        break;
    }
}
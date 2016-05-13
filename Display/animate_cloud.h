/* Created by sugar10w, 2016.5.10
 * 
 * 可以动画的点云
 */

#ifndef __ANIMATE_CLOUD_
#define __ANIMATE_CLOUD_

#include <string>
#include "common.h"

typedef enum{
    Stop,
    Appear,
    Disappear,
    Target,
    Plane,
    Counting,
    Bursting, 
    Zero
} AnimateStatus;

class AnimateCloud
{
public:
    AnimateCloud(PointCloudPtr cloud, std::string reg_name="cloud");
    ~AnimateCloud();

    
    inline PointCloudPtr getPointCloud() { return display_cloud_; }
    inline AnimateStatus getStatus() { return status_; }
    inline const std::string getRegName() { return reg_name_; }
    inline bool isVisible() {return visible_; }

    void setAnimateStatus(AnimateStatus status);
    void Refresh();
    

private:
    PointCloudPtr target_cloud_; 
    PointCloudPtr display_cloud_;
    PointCloudPtr plane_cloud_;

    AnimateStatus status_;
    int count_, max_count_;
    bool visible_;

    const int size_;
    const std::string reg_name_;
    const float k0, k1, k2;
};

#endif //__ANIMATE_CLOUD_
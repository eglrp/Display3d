/* Created by sugar10w, 2016.5.10
 * Last edited by sugar10w, 2016.5.13
 * 
 * 可以动画的点云
 */

#ifndef __ANIMATE_CLOUD_
#define __ANIMATE_CLOUD_

#include <string>
#include "common.h"

typedef enum{
    Stop,       // Shut down
    Appear,     // Appear instantly
    Disappear,  // Disappear instantly
    Target,     // Move to original position
    Plane,      // Move into a plane
    Counting,   // Appear in the order of index
    Bursting,   // Move out of the screen
    Zero        // Move to (0,0,0)
} AnimateStatus;   // 决定Refresh()时, 点云移动的目标位置以及移动方式

class AnimateCloud
{
public:
    AnimateCloud(PointCloudPtr cloud, std::string reg_name="cloud");
    ~AnimateCloud();
    
    inline PointCloudPtr getPointCloud() { return display_cloud_; }
    inline AnimateStatus getStatus() { return status_; }
    inline std::string getRegName() { return reg_name_; }
    inline bool isVisible() {return visible_; }

    void setAnimateStatus(AnimateStatus status);
    void Refresh();
    
private:
    PointCloudPtr target_cloud_;    // Oiginal position
    PointCloudPtr display_cloud_;   // Point cloud for display
    PointCloudPtr plane_cloud_;     // Plane

    AnimateStatus status_;
    int count_, max_count_;     // Count in Refresh() 
    bool visible_;              // The cloud would be removed from the viewer if not visible

    const int size_;                // Size of the point cloud
    const std::string reg_name_;    // Unique id string for the cloud in the viewer
    const float k0, k1, k2;         // Several constants to set the animation speed
};

#endif //__ANIMATE_CLOUD_
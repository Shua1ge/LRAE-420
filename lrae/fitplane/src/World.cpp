/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  Author: jianzhuozhu
 *  Date: 2021-7-24
 * 
 *  Modified by Qingchen Bi on 2022/11/05
 */

#include "plane.h"

using namespace std;
using namespace Eigen;

namespace FitPlane
{

World::World(const float &resolution, const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
{
    lowerbound_=INF*Vector3d::Ones(); 
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero(); 
    resolution_ = resolution;
    nh_ = nh;
    nh_private_ = nh_private;

    nh_private_.getParam("PointCloud_Map_topic", PointCloud_Map_topic);
    nh_private_.getParam("Grid_Map_topic", Grid_Map_topic);
    nh_private_.getParam("PointCloud_topic", PointCloud_topic);
    
    nh_private_.getParam("use_ex_range", use_ex_range_);
    nh_private_.getParam("ex_robot_back", ex_robot_back_);
    nh_private_.getParam("ex_robot_front", ex_robot_front_);
    nh_private_.getParam("ex_robot_right", ex_robot_right_);
    nh_private_.getParam("ex_robot_left", ex_robot_left_);
    nh_private_.getParam("fit_min_points", fit_min_points_);
    nh_private_.getParam("fit_z_half_extent", fit_z_half_extent_);
    nh_private_.getParam("fit_ransac", fit_ransac_);
    nh_private_.getParam("fit_ransac_thresh", fit_ransac_thresh_);
    nh_private_.getParam("fit_ransac_iters", fit_ransac_iters_);
    nh_private_.getParam("fit_ransac_min_ratio", fit_ransac_min_ratio_);
    nh_private_.getParam("grid_z_below", grid_z_below_);
    nh_private_.getParam("grid_z_above", grid_z_above_);
    nh_private_.getParam("fit_radius", fit_radius_);
    
    Grid_Map_pub = nh_.advertise<sensor_msgs::PointCloud2>(Grid_Map_topic, 1);
}

World::~World()
{
    clearMap();
}

void World::clearMap()
{
    if(has_map_)
    {
        for(int i=0;i < idx_count_(0);i++)
        {
            for(int j=0;j < idx_count_(1);j++)
            {
                delete[] grid_map_[i][j]; 
                grid_map_[i][j]=NULL; 
            }
            delete[] grid_map_[i];
            grid_map_[i]=NULL;
        }
        delete[] grid_map_;
        grid_map_=NULL;
    }
}

void World::initGridMap(const Vector3d &lowerbound,const Vector3d &upperbound)
{
    lowerbound_=lowerbound;
    upperbound_=upperbound;
    idx_count_=((upperbound_-lowerbound_)/resolution_).cast<int>()+Eigen::Vector3i::Ones(); 
    grid_map_=new bool**[idx_count_(0)];
    for(int i=0;i < idx_count_(0);i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j=0;j < idx_count_(1);j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool)); 
        }
    }
    has_map_=true; 
}

void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    if(cloud.points.empty())
    {
        ROS_ERROR("Can not initialize the map with an empty point cloud!");
        return;
    }
    cloud_near_.clear();
    GetRobotPosition();
    for(const auto&pt:cloud.points)
    {
        if(use_ex_range_)
        {
            if(pt.x < ex_robot_back_ || pt.x > ex_robot_front_ || pt.y < ex_robot_right_ || pt.y > ex_robot_left_)
                continue;            
        }
        if(abs(pt.x - ego_position_.x) > minrange_ || abs(pt.y - ego_position_.y) > minrange_)
            continue;
        else
        {
            cloud_near_.points.push_back(pt);        
        }
    }
    if(cloud_near_.points.empty())
    {
        rebuild_ok_ = false;
        return;
    }
    rebuild_ok_ = true;

    float minx = 10000, miny = 10000, minz = 10000;
    float maxx = -10000, maxy = -10000, maxz = -10000;
    for(const auto&pt:cloud_near_.points)
    {
        minx = std::min(minx, (float)pt.x); maxx = std::max(maxx, (float)pt.x);
        miny = std::min(miny, (float)pt.y); maxy = std::max(maxy, (float)pt.y);
        minz = std::min(minz, (float)pt.z); maxz = std::max(maxz, (float)(pt.z + 1.0));
    }

    // Optional vertical band clamp (keeps the voxel grid small when the cloud
    // reaches high structures).
    if(ego_position_.z - grid_z_below_ > minz)
        minz = ego_position_.z - grid_z_below_;
    if(ego_position_.z + grid_z_above_ < maxz)
        maxz = ego_position_.z + grid_z_above_;
    if(maxz <= minz + resolution_)
        maxz = minz + resolution_;

    // Allocate with a margin so the grid can be reused for several frames.
    const float margin_xy = 8.0f;
    const float margin_z = 1.0f;
    Eigen::Vector3d lo(minx - margin_xy, miny - margin_xy, minz - margin_z);
    Eigen::Vector3d hi(maxx + margin_xy, maxy + margin_xy, maxz + margin_z);
    // Quantise the allocation bounds: cloud bounds jitter by centimetres every
    // frame, and without this every frame would trigger a rebuild (measured
    // slower than the upstream per-frame allocation).
    lo = (lo.array() / 2.0).floor() * 2.0;
    hi = (hi.array() / 2.0).ceil() * 2.0;

    bool need_alloc = !grid_allocated_ || grid_map_ == NULL ||
        lo(0) < lowerbound_(0) || lo(1) < lowerbound_(1) || lo(2) < lowerbound_(2) ||
        hi(0) > upperbound_(0) || hi(1) > upperbound_(1) || hi(2) > upperbound_(2);
    if(need_alloc)
    {
        clearMap();
        lowerbound_ = lo;
        upperbound_ = hi;
        idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();
        for(int c = 0; c < 3; c++)
            if(idx_count_(c) < 1) idx_count_(c) = 1;
        grid_map_=new bool**[idx_count_(0)];
        for(int i = 0 ; i < idx_count_(0) ; i++)
        {
            grid_map_[i]=new bool*[idx_count_(1)];
            for(int j = 0 ; j < idx_count_(1) ; j++)
            {
                grid_map_[i][j]=new bool[idx_count_(2)];
                memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
            }
        }
        observed_.clear();
        grid_allocated_ = true;
    }
    else
    {
        // Reuse: clear only what the previous frame marked.
        for(const auto& idx : observed_)
            grid_map_[idx(0)][idx(1)][idx(2)] = true;
        observed_.clear();
    }
    has_map_=true;
}

void World::setObs(const Vector3d &point)
{   
    Vector3i idx=coord2index(point);
    if(!isInsideBorder(idx))
        return;
    if(grid_map_[idx(0)][idx(1)][idx(2)])
    {
        grid_map_[idx(0)][idx(1)][idx(2)] = false;
        observed_.push_back(idx);
    }
}

bool World::isFree(const Vector3d &point)
{
    Vector3i idx = coord2index(point);
    bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];
    return is_free;
}

Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    bool ifsuccess=false;

    if(x>=lowerbound_(0) && x<=upperbound_(0) && y>=lowerbound_(1) && y<=upperbound_(1))
    {
        for(float z = lowerbound_(2) ; z < upperbound_(2) ; z+=resolution_)
        {
            if( !isFree(x,y,z) && isFree(x,y,z+resolution_) )
            {
                *p_surface=Vector3d(x,y,z); 
                ifsuccess=true;
                break;
            }
        }
    }
    return ifsuccess;
}

bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
}
}


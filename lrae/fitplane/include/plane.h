/**
 *  Created by Qingchen Bi on 2022/11/05
 *  Several functions refer to PUTN
 */

#ifndef PLANE_H
#define PLANE_H

#include "World.h"
// #include "fitplane/PlaneMap.h"
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <deque>

namespace FitPlane
{

struct Plane
{
public: 
    Eigen::Vector3d init_coord; // 2D 
    std::vector<Eigen::Vector3d> plane_pts;
    Eigen::Vector3d normal_vector;
    float traversability = 100;
    float plane_height;
    float plane_angle;
}; 

class PlaneMap
{
public:
    PlaneMap(World* world, const float resolution);
    ~PlaneMap();
    bool InitPlaneMap();
    void clearPlaneMap();
    void PointCloudMapCallback(const sensor_msgs::PointCloud2& PointCloud_Map);

    bool init();
    bool getPlaneMap();
    Plane FitPlane(Eigen::Vector3d& p_surface,World* world,const double &radius);

    void visSurf(const PlaneMap &planemap, ros::Publisher* surf_vis_pub);
    bool pubPlaneGridMap(const PlaneMap &planemap);

    double getAngle(Eigen::Vector3d &plane_vector);

    Plane** Plane_Map_=NULL; 
    // fitplane::PlaneMap PlaneGridMap_; 
    bool has_PlaneMap_=false;
    float resolution_;
    Eigen::Vector3i index_num_;
    Eigen::Vector3d leftdownbound_;
    Eigen::Vector3d rightupbound_;
    World* world_ = NULL;

    // For flat terrain
    // float max_angle_ = 60.0; 
    // float max_flatness_ = 0.7;
    // float w1_ = 0.9;
    
    // For uneven terrain
    float max_angle_ = 40.0; 
    float max_flatness_ = 0.01;
    float w1_ = 0.8;
    // Post-process dilation: every free cell 8-adjacent to an occupied cell
    // is rewritten to flatThre_ (99), which exploration_map_merge treats as
    // an obstacle.  This inflates every obstacle by one 0.3 m cell and is
    // the main source of black cells hugging the traversed path when the
    // vegetation line is within ~0.6 m.  Measured on the 2026-11-05 bag:
    // 91% of the occupied cells within 1 m of the path were first written
    // as 99, not as a genuine 100.  Set false to keep the map geometric.
    bool occ_dilate_ = true;
    // Same 0.3 m ring, but only for /local_traversibility_ponit_cloud,
    // i.e. the local planner's obstacle map.  The published occupancy
    // grid stays geometric, so RViz/globalMap show real obstacle cells
    // while localPlanner keeps its clearance margin.
    bool occ_dilate_local_ = true;
    // Temporal accumulation: fit on the union of the last N clouds.
    // /cloud_registered is already in the map frame, so a 3-frame union
    // needs no pose bookkeeping; at 0.5 m/s the robot moves 0.15 m, which
    // is below the 0.3 m cell size.  Density x N is what makes a 3-point
    // "plane" (degenerate: SVD residual is exactly 0) rare instead of
    // routine.  1 = upstream behaviour.
    int accum_frames_ = 1;
    // Ground-height gate: a flat patch is not automatically walkable.
    // A cell whose surface sits more than this far above the median
    // surface of its fitted neighbours is a hedge/roof/canopy top and is
    // demoted to obstacle.  0 disables (upstream behaviour).  On a
    // uniform slope the median is symmetric, so slopes are unaffected.
    float ground_gate_height_ = 0.0f;
    int ground_gate_radius_cells_ = 3;
    int ground_gate_min_neighbors_ = 5;
    int ground_gated_ = 0;
    std::deque<pcl::PointCloud<pcl::PointXYZ>> recent_clouds_;

    ros::Publisher plane_grid_map_pub_;

    int width_;
    int height_;
    nav_msgs::OccupancyGrid plane_Occmap_;
    nav_msgs::OccupancyGrid Original_plane_Occmap_;
    ros::Publisher plane_OccMap_pub_;
    ros::Publisher trav_cloud_pub;

    int occThre_ = 100;
    int flatThre_ = 99;
    int nobs_ = 120;

    // Per-frame diagnostics, counted over the same fixed +-20 m window that
    // getPlaneMap() processes, so frames are comparable.
    //   surface_failed   : project2surface() found no ground candidate
    //   fit_insufficient : candidate found, fewer than fit_min_points voxels
    //   fit_success      : plane fitted (free or obstacle)
    //   free_cells / occupied_cells : final classification in the window
    // Neighborhood surface recovery (experiment C): when the exact 0.1 m
    // column has no surface, gather surfaces from the 3x3 neighbouring
    // 0.3 m cells and fit them, with collinearity and height-jump guards.
    bool neighbor_recover_ = false;
    int neighbor_min_points_ = 3;
    int neighbor_radius_cells_ = 1;  // 1 => 3x3, 2 => 5x5, 3 => 7x7
    float neighbor_min_eig_ratio_ = 0.1f;
    float neighbor_max_z_span_ = 0.15f;

    int surface_failed_ = 0;
    int fit_insufficient_ = 0;
    int fit_success_ = 0;
    int free_cells_ = 0;
    int occupied_cells_ = 0;
    int window_cells_ = 0;
    int neighbor_recovered_ = 0;
    // Rejection reasons for the neighbourhood path, plus the denominator:
    // cells that have at least one observed surface within the search ring.
    int nb_no_points_ = 0;
    int nb_collinear_ = 0;
    int nb_zjump_ = 0;
    int nb_recoverable_ = 0;
    double last_frame_ms_ = 0.0;
    double last_rebuild_ms_ = 0.0;
    ros::Publisher diag_pub_;
    void publishDiagnostics();
    Plane fitFromPoints(const std::vector<Eigen::Vector3d>& pts);
    void windowBounds(int& lowerx, int& lowery, int& upperx, int& uppery);
    
    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + leftdownbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }
    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-leftdownbound_)/resolution_).cast<int>();            
        return index;
    }
    bool isInBorder(const int& x, const int& y)
    {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros::Publisher pubExploredArea;
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
};

}

#endif
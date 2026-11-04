/**
 *  Created by Qingchen Bi on 2022/11/05
 */

#include "plane.h"
#include <Eigen/Eigenvalues>
#include <vector>
#include <cmath>
#include <std_msgs/Int32MultiArray.h>

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

int dx_[4] = {-1, 0, 1, 0};
int dy_[4] = {0, 1, 0, -1};

// FILE* file = fopen("./Analysis.txt", "w+");

namespace FitPlane
{
    PlaneMap::PlaneMap(World* world, const float resolution)
    {
        world_ = world;
        resolution_ = resolution;
        init();
    }

    void PlaneMap::PointCloudMapCallback(const sensor_msgs::PointCloud2& PointCloud_Map)
    {
        // Slam-sim-out or Fast-Lio
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(PointCloud_Map, cloud);
        world_->initGridMap(cloud); 

        for (const auto& pt : world_->cloud_near_)
        {
            Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
            world_->setObs(obstacle);
        }
        // visualization::visWorld(world_, &world_->Grid_Map_pub); 
        InitPlaneMap();
        
        timeval start;
        gettimeofday(&start, NULL);
        
        getPlaneMap();
        // visSurf(*this, &world_->Plane_Map_pub);
        pubPlaneGridMap(*this);

        timeval end;
        gettimeofday(&end, NULL);
        double ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
        last_frame_ms_ = ms;
        publishDiagnostics();
    }

    bool PlaneMap::init()
    {
        world_->nh_private_.getParam("Plane_Map_topic", world_->Plane_Map_topic);
        world_->Plane_Map_pub = world_->nh_.advertise<sensor_msgs::PointCloud2>(world_->Plane_Map_topic, 1);
        world_->PointCloud_Map_sub = world_->nh_.subscribe(world_->PointCloud_Map_topic, 10, &FitPlane::PlaneMap::PointCloudMapCallback, this);

        world_->nh_private_.getParam("neighbor_recover", neighbor_recover_);
        world_->nh_private_.getParam("neighbor_min_points", neighbor_min_points_);
        world_->nh_private_.getParam("neighbor_radius_cells", neighbor_radius_cells_);
        world_->nh_private_.getParam("neighbor_min_eig_ratio", neighbor_min_eig_ratio_);
        world_->nh_private_.getParam("neighbor_max_z_span", neighbor_max_z_span_);
        plane_OccMap_pub_ = world_->nh_.advertise<nav_msgs::OccupancyGrid>("plane_OccMap", 1);
        diag_pub_ = world_->nh_.advertise<std_msgs::Int32MultiArray>("fitplane_diagnostics", 10);
        trav_cloud_pub = world_->nh_.advertise<sensor_msgs::PointCloud2>("local_traversibility_ponit_cloud", 1);
        pubExploredArea = world_->nh_.advertise<sensor_msgs::PointCloud2> ("/explored_areas", 5);
        exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);

        return true;        
    }

    PlaneMap::~PlaneMap()
    {
        clearPlaneMap();
    }
    
    void PlaneMap::clearPlaneMap()
    {
        if(this->has_PlaneMap_)
        {
            for(int i=0;i < index_num_(0);i++)
            {
                if(Plane_Map_[i] != NULL)
                {
                    delete[] Plane_Map_[i]; 
                    Plane_Map_[i]=NULL; 
                }
            }
            if(Plane_Map_ !=NULL)
            {
                delete[] Plane_Map_;
                Plane_Map_=NULL;
            }            
        }

    }

    bool PlaneMap::InitPlaneMap()
    {
        if(this->has_PlaneMap_)
            clearPlaneMap();

        if(world_ != NULL)
        {
            leftdownbound_ = world_->getLowerBound();
            rightupbound_ = world_->getUpperBound();
            index_num_ = ((rightupbound_ - leftdownbound_) / resolution_).cast<int>() + Eigen::Vector3i::Ones(); 
            Plane_Map_ = new Plane*[index_num_(0)];
            for(int i = 0; i < index_num_(0); i++)
            {
                Plane_Map_[i] = new Plane[index_num_(1)];
            }
            this->has_PlaneMap_ = true; 
        }
        else
        {
            ROS_ERROR("No world ! ");
            this->has_PlaneMap_ = false;
        }

        return this->has_PlaneMap_;
    }

    void PlaneMap::windowBounds(int& lowerx, int& lowery, int& upperx, int& uppery)
    {
        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_);
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);
        lowerx = std::max(0, r_x - int(20.0 / resolution_));
        lowery = std::max(0, r_y - int(20.0 / resolution_));
        upperx = std::min(index_num_(0), r_x + int(20.0 / resolution_));
        uppery = std::min(index_num_(1), r_y + int(20.0 / resolution_));
    }

    Plane PlaneMap::fitFromPoints(const std::vector<Eigen::Vector3d>& pts)
    {
        Plane tmp_Plane;  // traversability stays 100 (not fitted) by default
        if(pts.size() < 3)
            return tmp_Plane;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for(const auto& q : pts) center += q;
        center /= (double)pts.size();
        Eigen::MatrixXd A(pts.size(), 3);
        for(size_t i = 0; i < pts.size(); i++)
            A.row(i) = pts[i] - center;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
        tmp_Plane.normal_vector = svd.matrixV().col(2);
        tmp_Plane.init_coord = center;
        double angle_ = getAngle(tmp_Plane.normal_vector);
        angle_ = angle_ * 180.0 / PI;
        double D = tmp_Plane.normal_vector.dot(center);
        tmp_Plane.plane_height = D;
        tmp_Plane.plane_angle = angle_;
        double sumd = 0.0;
        for(size_t i = 0; i < pts.size(); i++)
        {
            double dval = tmp_Plane.normal_vector.dot(A.row(i));
            sumd += dval * dval;
        }
        float flatness = sumd / pts.size();
        tmp_Plane.plane_pts = pts;
        if(angle_ >= max_angle_ || flatness >= max_flatness_)
            tmp_Plane.traversability = 1;
        else
            tmp_Plane.traversability =
                w1_ * (angle_ / max_angle_) + (1 - w1_) * (flatness / max_flatness_);
        return tmp_Plane;
    }

    bool PlaneMap::getPlaneMap()
    {
        surface_failed_ = fit_insufficient_ = fit_success_ = 0;
        neighbor_recovered_ = 0;
        nb_no_points_ = nb_collinear_ = nb_zjump_ = nb_recoverable_ = 0;
        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_); 
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);
        int lowerx = std::max(0, r_x - int(20.0 / resolution_));
        int lowery = std::max(0, r_y - int(20.0 / resolution_));
        int upperx = std::min(index_num_(0), r_x + int(20.0 / resolution_));
        int uppery = std::min(index_num_(1), r_y + int(20.0 / resolution_));
        int wx = std::max(0, upperx - lowerx);
        int wy = std::max(0, uppery - lowery);
        // Padded per-cell surface heights for the exact-column pass, so the
        // neighbourhood search does not re-scan voxel columns (~50x cheaper).
        int PAD = std::max(1, neighbor_recover_ ? neighbor_radius_cells_ : 1);
        int WX = wx + 2 * PAD, WY = wy + 2 * PAD;
        std::vector<float> surf(WX * WY, NAN);
        std::vector<char> has(WX * WY, 0);
        for(int i = 0; i < wx; i++)
        {
            for(int j = 0; j < wy; j++)
            {
                Eigen::Vector3d tmp_P(PlaneMap::index2coord(Eigen::Vector3i(lowerx + i, lowery + j, 0)));
                Eigen::Vector3d p_sur;
                if(world_->project2surface(tmp_P(0), tmp_P(1), &p_sur))
                {
                    int idx = (i + PAD) + (j + PAD) * WX;
                    surf[idx] = p_sur(2);
                    has[idx] = 1;
                }
            }
        }
        for(int i = 0; i < wx; i++)
        {
            for(int j = 0; j < wy; j++)
            {
                int gi = lowerx + i, gj = lowery + j;
                int idx = (i + PAD) + (j + PAD) * WX;
                if(has[idx])
                {
                    Eigen::Vector3d tmp_P(PlaneMap::index2coord(Eigen::Vector3i(gi, gj, 0)));
                    Eigen::Vector3d p_sur(tmp_P(0), tmp_P(1), surf[idx]);
                    Plane fitted = FitPlane(p_sur, world_, world_->fit_radius_);
                    Plane_Map_[gi][gj] = fitted;
                    if(fitted.traversability == 100)
                        fit_insufficient_++;
                    else
                        fit_success_++;
                    continue;
                }
                if(!neighbor_recover_)
                {
                    surface_failed_++;
                    continue;
                }
                // ---- experiment C: neighbourhood surface recovery ----
                std::vector<Eigen::Vector3d> pts;
                int NR = std::max(1, neighbor_radius_cells_);
                for(int di = -NR; di <= NR; di++)
                {
                    for(int dj = -NR; dj <= NR; dj++)
                    {
                        if(di == 0 && dj == 0) continue;
                        int ni = i + PAD + di, nj = j + PAD + dj;
                        if(ni < 0 || nj < 0 || ni >= WX || nj >= WY) continue;
                        if(!has[ni + nj * WX]) continue;
                        Eigen::Vector3d np = PlaneMap::index2coord(
                            Eigen::Vector3i(lowerx + i + di, lowery + j + dj, 0));
                        pts.push_back(Eigen::Vector3d(np(0), np(1), surf[ni + nj * WX]));
                    }
                }
                if(!pts.empty())
                    nb_recoverable_++;
                if((int)pts.size() < neighbor_min_points_)
                {
                    nb_no_points_++;
                    surface_failed_++;
                    continue;
                }
                // guard 1: height jump (curb/step/edge) across the 3x3 ring
                float zmin = 1e9f, zmax = -1e9f;
                for(const auto& q : pts) { zmin = std::min(zmin, (float)q(2)); zmax = std::max(zmax, (float)q(2)); }
                if(zmax - zmin > neighbor_max_z_span_)
                {
                    nb_zjump_++;
                    surface_failed_++;
                    continue;
                }
                // guard 2: points must not be collinear / one-sided (a single
                // lidar ring crossing the cell would otherwise be fitted)
                Eigen::Vector2d c2 = Eigen::Vector2d::Zero();
                for(const auto& q : pts) c2 += q.head<2>();
                c2 /= (double)pts.size();
                Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
                for(const auto& q : pts)
                {
                    Eigen::Vector2d d = q.head<2>() - c2;
                    cov += d * d.transpose();
                }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
                double lmin = es.eigenvalues()(0), lmax = es.eigenvalues()(1);
                if(lmax <= 1e-9 || lmin / lmax < neighbor_min_eig_ratio_)
                {
                    nb_collinear_++;
                    surface_failed_++;
                    continue;
                }
                Plane fitted = fitFromPoints(pts);
                Plane_Map_[gi][gj] = fitted;
                if(fitted.traversability == 100)
                    fit_insufficient_++;
                else
                {
                    fit_success_++;
                    neighbor_recovered_++;
                }
            }
        }
        return true;
    }


    Plane PlaneMap::FitPlane(Eigen::Vector3d& p_surface,World* world,const double &radius)
    {
        Plane tmp_Plane;
        tmp_Plane.init_coord = Eigen::Vector3d(p_surface(0), p_surface(1), p_surface(2));        

        Eigen::Vector3d ball_center = world->coordRounding(p_surface);
        float resolution = world->getResolution();
        int fit_num=static_cast<int>(radius/resolution);
        Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> vac(2*fit_num+1,2*fit_num+1);
        int vac_cout_init=(2*fit_num+1)*(2*fit_num+1);
        
        for(int i = -fit_num;i <= fit_num;i++)
        {
            for(int j = -fit_num;j <= fit_num;j++)
            {
                vac(i+fit_num,j+fit_num)=false;
                for(int k = -10;k <= 10;k++)
                { 
                    Eigen::Vector3d point=ball_center+resolution*Eigen::Vector3d(i,j,k); 
                    
                    if(world->isInsideBorder(point) && !world->isFree(point))
                    {
                        tmp_Plane.plane_pts.push_back(point);
                        if(!vac(i+fit_num,j+fit_num))
                        {
                            vac(i+fit_num,j+fit_num)=true;
                            vac_cout_init--;
                        }
                    }
                }
            }
        }
        
        size_t pt_num=tmp_Plane.plane_pts.size();

        if((int)pt_num < world->fit_min_points_)
            return tmp_Plane;

        Eigen::Vector3d center;
        for(const auto&pt:tmp_Plane.plane_pts) center += pt; 
        center /= pt_num; 
        Eigen::MatrixXd A(pt_num,3);
        for(size_t i = 0; i < pt_num; i++) A.row(i)=tmp_Plane.plane_pts[i]-center; 

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeFullV);
        tmp_Plane.normal_vector=svd.matrixV().col(2); 
        double angle_ = getAngle(tmp_Plane.normal_vector);
        angle_ = angle_ * 180.0 / PI;

        double D = tmp_Plane.normal_vector(0) * center(0) + tmp_Plane.normal_vector(1) * center(1) + tmp_Plane.normal_vector(2) * center(2);
        double H = D - (tmp_Plane.normal_vector(0) * p_surface(0) + tmp_Plane.normal_vector(1) * p_surface(1));
        H = H / tmp_Plane.normal_vector(2); 

        float flatness = 0.0;
        double dval = 0.0;
        double sumd = 0.0;
        for(size_t i = 0; i < pt_num; i++)
        {
           dval = tmp_Plane.normal_vector.dot(A.row(i));
           sumd = sumd + dval * dval;
        }
        flatness = sumd / pt_num;   

        if(angle_ >= max_angle_ || flatness >= max_flatness_)   
            tmp_Plane.traversability = 1; 
        else 
            tmp_Plane.traversability = w1_ * (angle_ / max_angle_) + (1 - w1_) * (flatness / max_flatness_);

        tmp_Plane.plane_height = H;
        tmp_Plane.plane_angle = angle_;

        return tmp_Plane;
    }

    void PlaneMap::visSurf(const PlaneMap &planemap, ros::Publisher* surf_vis_pub)
    {
        if (surf_vis_pub == NULL)
            return;
        pcl::PointCloud<pcl::PointXYZRGB> surf_point;
        pcl::PointXYZRGB pt;

        for(int i = 0; i < planemap.index_num_[0]; i++)
        {
            for(int j = 0; j < planemap.index_num_[1]; j++)
            {
                if(planemap.Plane_Map_[i][j].traversability != 100)
                {
                    for(const auto& point : planemap.Plane_Map_[i][j].plane_pts)
                    {
                        pt.x = point(0);
                        pt.y = point(1);
                        pt.z = point(2);
                        pt.r = pt.g = pt.b = 0;
                        pt.a = 1.0f;

                        surf_point.push_back(pt);        
                    }
                }
            }
        }

        surf_point.width = surf_point.points.size();
        surf_point.height = 1;
        surf_point.is_dense = true;

        sensor_msgs::PointCloud2 map_vis;
        pcl::toROSMsg(surf_point, map_vis);

        map_vis.header.frame_id = "map";
        surf_vis_pub->publish(map_vis);
    }

    void PlaneMap::publishDiagnostics()
    {
        int lowerx, lowery, upperx, uppery;
        windowBounds(lowerx, lowery, upperx, uppery);
        free_cells_ = occupied_cells_ = 0;
        window_cells_ = std::max(0, upperx - lowerx) * std::max(0, uppery - lowery);
        for(int i = lowerx; i < upperx; i++)
        {
            for(int j = lowery; j < uppery; j++)
            {
                float t = Plane_Map_[i][j].traversability;
                if(t == 100)      // not fitted in this frame
                    continue;
                if(100.0 * t < 50.0)
                    free_cells_++;
                else
                    occupied_cells_++;
            }
        }
        std_msgs::Int32MultiArray msg;
        msg.data = {surface_failed_, fit_insufficient_, fit_success_,
                    free_cells_, occupied_cells_, window_cells_,
                    neighbor_recovered_};
        if(diag_pub_)
            diag_pub_.publish(msg);
        ROS_INFO_THROTTLE(1.0,
            "[fitplane] fit_ms=%.1f window=%d surface_failed=%d fit_insufficient=%d "
            "fit_success=%d neighbor_recovered=%d free=%d occupied=%d "
            "recoverable=%d nb_no_points=%d nb_collinear=%d nb_zjump=%d",
            last_frame_ms_, window_cells_, surface_failed_, fit_insufficient_,
            fit_success_, neighbor_recovered_, free_cells_, occupied_cells_,
            nb_recoverable_, nb_no_points_, nb_collinear_, nb_zjump_);
    }

    bool PlaneMap::pubPlaneGridMap(const PlaneMap &planemap)
    {
        plane_Occmap_.info.height = planemap.index_num_(1) * int(resolution_ / resolution_); // 0.5
        plane_Occmap_.info.width = planemap.index_num_(0) * int(resolution_ / resolution_);// 0.5
        plane_Occmap_.info.resolution = resolution_;// 0.5
        plane_Occmap_.header.frame_id = "map";
        plane_Occmap_.info.origin.position.x = leftdownbound_(0);
        plane_Occmap_.info.origin.position.y = leftdownbound_(1);
        plane_Occmap_.data.clear();
        std::vector<int8_t> tmp(plane_Occmap_.info.height * plane_Occmap_.info.width, -1);
        plane_Occmap_.data = tmp;

        for(unsigned int i = 0; i < plane_Occmap_.info.width; i++)
        {
            for(unsigned int j = 0; j < plane_Occmap_.info.height; j++)
            {   
                double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                int l_x = CONTXY2DISC(x - leftdownbound_(0), resolution_); 
                int l_y = CONTXY2DISC(y - leftdownbound_(1), resolution_);
            
                if(l_x >= 0 && l_x < planemap.index_num_(0) && l_y >= 0 && l_y < planemap.index_num_(1))
                {
                    if(planemap.Plane_Map_[l_x][l_y].traversability != 100)
                    {
                        if(planemap.Plane_Map_[l_x][l_y].traversability == 1)// 0.9 && planemap.Plane_Map_[i][j].traversability <= 1.0)
                        {
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j]= occThre_;
                        }
                        else if(planemap.Plane_Map_[l_x][l_y].traversability == 10)// 0.9 && planemap.Plane_Map_[i][j].traversability <= 1.0)
                        {
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j]= nobs_;
                        }
                        else
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j] = 100 * planemap.Plane_Map_[l_x][l_y].traversability;   

                    }
                }


            }
        }

        Original_plane_Occmap_ = plane_Occmap_;
        int d_x8[8] = {-1, 0, 0, 1, 1, -1, 1, -1};
        int d_y8[8] = {0, 1, -1, 0, 1, -1, -1, 1};
        width_ = plane_Occmap_.info.width;
        height_ = plane_Occmap_.info.height;
        int sum = 0;
        int count = 0;
        for(int i = 0; i < width_; i++)
        {
            for(int j = 0; j < height_; j++)
            {

                if(Original_plane_Occmap_.data[i + j * width_] == occThre_) 
                {
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                            if(Original_plane_Occmap_.data[x_tmp + y_tmp * width_] == -1)
                            {
                                sum = sum + 0;
                                count++;
                            }
                            else if(Original_plane_Occmap_.data[x_tmp + y_tmp * width_] != occThre_)
                            {
                                sum = Original_plane_Occmap_.data[x_tmp + y_tmp * width_] + sum;
                                count++;
                            }
                        }
                    }
                    if(count == 8)
                    {
                        plane_Occmap_.data[i + j * width_] = (plane_Occmap_.data[i + j * width_] + sum ) / (count + 1);
                    }
                    sum = 0;    
                    count = 0;                    
                }
            }
        }
        for(int i = 0; i < width_; i++)
        {
            for(int j = 0; j < height_; j++)
            {
                if(plane_Occmap_.data[i + j * width_] == occThre_ || plane_Occmap_.data[i + j * width_] == nobs_) // 
                {
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                            if(plane_Occmap_.data[x_tmp + y_tmp * width_] < flatThre_)
                            {
                                if(plane_Occmap_.data[i + j * width_] == occThre_)
                                    plane_Occmap_.data[x_tmp + y_tmp * width_] = flatThre_;
                                else
                                {
                                    plane_Occmap_.data[x_tmp + y_tmp * width_] =  nobs_ - 1;
                                }
                            }
                        }
                    }
                } 
            }
        }
        plane_Occmap_.header.stamp = ros::Time::now();
        plane_OccMap_pub_.publish(plane_Occmap_);

        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_); 
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr trav_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for(int i = r_x - 12; i < r_x + 13; i++)
        {
            for(int j = r_y - 12; j < r_y + 13; j++)
            {
                if(isInBorder(i, j))
                {
                    if(plane_Occmap_.data[i + j * width_] >= 99)
                    {
                        double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                        double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                        pcl::PointXYZI reg_point;
                        reg_point.x = x;
                        reg_point.y = y;
                        reg_point.z = world_->ego_position_.z + 0.1 * 6;
                        reg_point.intensity = 1;
                        trav_point_cloud->points.push_back(reg_point);
                        
                    }
                    else
                    {
                        double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                        double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                        pcl::PointXYZI reg_point;
                        reg_point.x = x;
                        reg_point.y = y;
                        reg_point.z = world_->ego_position_.z - 0.1;
                        reg_point.intensity = 0.2;
                        trav_point_cloud->points.push_back(reg_point);
                    }
                }
            }
        }
        sensor_msgs::PointCloud2 trav_point_cloud_msg;
        pcl::toROSMsg(*trav_point_cloud, trav_point_cloud_msg);
        trav_point_cloud_msg.header.stamp= ros::Time(0);
        trav_point_cloud_msg.header.frame_id = "map";
        trav_cloud_pub.publish(trav_point_cloud_msg);// publish downsample point cloud

        return true;
    }

    double PlaneMap::getAngle(Eigen::Vector3d &plane_vector)
    {
        Eigen::Vector3d n1(0,0,1);
	    double cos_ = abs(n1(0) * plane_vector(0) + n1(1) * plane_vector(1) + n1(2) * plane_vector(2)) / 
			    (sqrt(n1(0) * n1(0) + n1(1) * n1(1) + n1(2) * n1(2)) * 
         		sqrt(plane_vector(0) * plane_vector(0) + plane_vector(1) * 
	    		plane_vector(1) + plane_vector(2) * plane_vector(2)));
        double angle = std::acos(cos_);
	return angle;
    }

}

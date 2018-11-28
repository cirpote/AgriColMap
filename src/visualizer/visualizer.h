#pragma once
#include <uav_ugv_collaboration_module/utils.h>
 
class PointCloudViz{

    public:
        PointCloudViz();
        ~PointCloudViz();

        void inline spinOnce(){ viewer->spinOnce(); }

        void setViewerPosition(const float& x, const float& y, const float& z,
                               const float& view_x, const float& view_y, const float& view_z);
        void setViewerBackground(const float& r, const float& g, const float& b);

        void removeCloud(const std::string &cloud_to_show);

        void showCloud(const PCLPointCloudXYZRGB::Ptr points,
                       const std::string &cloud_to_show, const int& size = 1);
        void showTransparentCloud(const std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& points,
                       const std::string &cloud_to_show, const int& size = 1);
        void spingUntilDeath();

    private:
        pcl::visualization::PCLVisualizer* viewer;

};

#pragma once

#include "defs.h"

std::shared_ptr<PCLPointCloud> pcl_point_cloud( new PCLPointCloud() );
std::shared_ptr<PCLVoxelGrid> pcl_voxel_grid( new PCLVoxelGrid() );
std::shared_ptr<PCLkdTreeFlann> pcl_kd_tree( new PCLkdTreeFlann() );
std::shared_ptr<PCLPointCloudXYZ> pcl_point_cloud_xyz( new PCLPointCloudXYZ() );
VectorXYZRGB vectorXYZRGB();

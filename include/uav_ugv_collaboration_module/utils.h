#pragma once
#include "defs.h"

Vector3 inline toVector3(const std::vector<float>& vector){

    return Vector3( vector[0], vector[1], vector[2]);
}

float inline computeAngle(const Matrix3& R) {
    float rotation_trace = R.trace();
    return acos(std::min(float(1), std::max(float(-1), (rotation_trace - 1)/2)));
}

void inline ExitWithErrorMsg(const std::string& msg){
    std::cerr << FRED( "ERROR: " ) << msg << "\n";
    std::exit(1);
}

std::vector<double> vectorFromString(const std::string& str);

void AffineTransformFromString(const std::string& str,
                               Eigen::Matrix3f& R,
                               Eigen::Vector3f& t,
                               Eigen::Vector2f& scale);

Vector3 getScaleFromAffineMatrix(const Matrix3& aff);

unsigned char computeExGforPoint(const pcl::PointXYZRGB& pt,
                                 std::vector< pcl::PointXYZRGB,
                                 Eigen::aligned_allocator<pcl::PointXYZRGB> >& PointCloudFiltered,
                                 const Vector3i& color);

pcl::PointXYZRGB getHighestPoint(std::vector<int>& pt_list,
                                 std::vector< pcl::PointXYZRGB,
                                 Eigen::aligned_allocator<pcl::PointXYZRGB> >& pt_cloud);

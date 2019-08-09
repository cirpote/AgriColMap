#pragma once

// std c++ Headers
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>

// Ros Headers
#include <opencv2/opencv.hpp>

// PCL Headers
#ifdef BUILD_WITH_PCL
    #include "io/include/pcl/io/ply_io.h"
    #include "visualization/include/pcl/visualization/pcl_visualizer.h"
    #include "common/include/pcl/common/common.h"
    #include "filters/include/pcl/filters/voxel_grid.h"
    #include "segmentation/include/pcl/segmentation/sac_segmentation.h"
    #include "common/include/pcl/common/transforms.h"
#else
    #include <pcl/io/ply_io.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <pcl/common/common.h>
    #include <pcl/filters/voxel_grid.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/common/transforms.h>
#endif

// CPM Header
#include "../../CPM/CPM.h"
#include "../../CPM/OpticFlowIO.h"

// cpd Header
#include "../../cpd/include/cpd/affine.hpp"
#include "../../cpd/include/cpd/rigid.hpp"

// GoICP Header
#include "../../src_GoICP/jly_goicp.h"

#include "../../srrg_core/src/srrg_kdtree/kd_tree.hpp"

/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x)	"\x1B[1m" x RST
#define UNDL(x)	"\x1B[4m" x RST



struct MspIntrnscsParams{

    float px, py;
    float Aff;
    float p1, p2;

    MspIntrnscsParams() : px(0.f), py(0.f), Aff(0.f), p1(0.f), p2(0.f) {};
    
    void print(){
        cout.precision(10);
        cout << "Principal Point: " << px << " " << py << "\n\n";
        cout << "Affine Value: " << Aff<< "\n\n";
        cout << "Polynomial Coeffs:\n" << p1 << " " << p2 << "\n\n";
    }
};

struct MspCalibCamParams{

    char img[50];
    int img_width, img_height;
    Eigen::Vector3d cam_t;
    Eigen::Matrix3d cam_R;

    MspCalibCamParams() : cam_t(Eigen::Vector3d::Zero()), cam_R(Eigen::Matrix3d::Identity()) {};
    
    void print(){
        cout.precision(10);
        cout << "Image size: " << img_width << " " << img_height << "\n\n";
        cout << "Camera Position:\n" << cam_t.transpose() << "\n\n";
        cout << "Camera Rotation Matrix:\n" << cam_R << "\n\n";
    }
};






// OLD STUFF

// Eigen typedefs
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3i;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Quaternionf Quat;
typedef Eigen::Isometry3f Transform;


















enum MatchingType {
    SURF,
    SIFT,
    ORB,
    FAST_BRIEF
};

struct GroundTruth{

    Matrix3 _Rgt;
    Vector3 _tgt;
    Vector2 _rel_scl;

    ~GroundTruth(){}
    GroundTruth() {
        _Rgt.setIdentity();
        _tgt.setZero();
        _rel_scl.setZero();
    }
    GroundTruth( const Matrix3& R,
                  const Vector3& t,
                  const Vector2& scl ){
        _Rgt = R;
        _tgt = t;
        _rel_scl = scl;
    }


};

struct _PointData{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    uint _ExG;
    uint _elev;

    _PointData() { _ExG = 0; _elev = 0;}
}; 

typedef std::unordered_map<std::string, const boost::shared_ptr<GroundTruth> > GroundTruthUnorderedMap;
typedef std::unordered_map<std::string, const boost::shared_ptr<Transform> > TransformUnorderedMap;

using namespace pcl::io;

// PCL typedefs
typedef pcl::PointXYZRGB PCLptXYZRGB;
typedef pcl::PointXYZ PCLptXYZ;
typedef pcl::PointCloud<PCLptXYZRGB> PCLPointCloudXYZRGB;
typedef pcl::PointCloud<PCLptXYZ> PCLPointCloudXYZ;
typedef std::unordered_map< std::string, PCLPointCloudXYZRGB::Ptr> PCLXYZRGB_unMap;
typedef pcl::SACSegmentation<PCLptXYZ> PCLsegmentationXYZ;
typedef pcl::SACSegmentation<PCLptXYZRGB> PCLsegmentationXYZRGB;
typedef pcl::VoxelGrid<pcl::PointXYZRGB> PCLvoxelGridXYZRGB;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> PCLXYZRGB_viz;
typedef srrg_core::KDTree<float, 2>::VectorTD KDTreeXYpoint;
typedef srrg_core::KDTree<float, 2>::VectorTDVector KDTreeXYvector;
typedef srrg_core::KDTree<float, 2> KDTreeXY;
typedef srrg_core::KDTree<float, 3>::VectorTD KDTreeXYZpoint;
typedef srrg_core::KDTree<float, 3>::VectorTDVector KDTreeXYZvector;
typedef srrg_core::KDTree<float, 3> KDTreeXYZ;



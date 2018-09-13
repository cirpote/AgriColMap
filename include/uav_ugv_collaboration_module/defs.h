#pragma once

// std c++ Headers
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>

// Ros Headers
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>

// PCL Headers
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>


// CPM Header
#include "../../CPM/CPM.h"
#include "../../CPM/OpticFlowIO.h"

// cpd Header
#include "../../cpd/include/cpd/affine.hpp"
#include "../../cpd/include/cpd/rigid.hpp"

#include "../../src_GoICP/jly_goicp.h"
#include "opencv2/xfeatures2d.hpp"

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

typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3i;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Quaternionf Quat;
typedef Eigen::Isometry3f Transform;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;

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

#pragma once

// std c++ Headers
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <thread>

// OpenCV Headers
#include <opencv2/opencv.hpp>

// Open3D Headers
#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

// CPM Headers
#include "../../CPM/CPM.h"
#include "../../CPM/OpticFlowIO.h"

// cpd Headers
#include "../../cpd/include/cpd/affine.hpp"
#include "../../cpd/include/cpd/rigid.hpp"

// Srrg Headers
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

// Eigen typedefs
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3i Vector3i;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Quaternionf Quat;
typedef Eigen::Isometry3d Transform;

enum MatchingType {
    SURF,
    SIFT,
    ORB,
    FAST_BRIEF
};

struct GroundTruth{

    Matrix3d _Rgt;
    Vector3d _tgt;
    Vector2d _rel_scl;

    ~GroundTruth(){}
    GroundTruth() {
        _Rgt.setIdentity();
        _tgt.setZero();
        _rel_scl.setZero();
    }
    GroundTruth( const Matrix3d& R,
                  const Vector3d& t,
                  const Vector2d& scl ){
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
typedef std::unordered_map<std::string, std::shared_ptr<open3d::PointCloud> > PCLXYZRGB_Map;
typedef std::unordered_map<std::string, const boost::shared_ptr<Transform> > TransformUnorderedMap;

typedef srrg_core::KDTree<double, 2>::VectorTDVector KDTreeXYvector;
typedef srrg_core::KDTree<double, 2> KDTreeXY;
typedef srrg_core::KDTree<double, 2>::VectorTD KDTreeXYpoint;
typedef srrg_core::KDTree<double, 3>::VectorTD KDTreeXYZpoint;
typedef srrg_core::KDTree<double, 3>::VectorTDVector KDTreeXYZvector;
typedef srrg_core::KDTree<double, 3> KDTreeXYZ;

#pragma once
#include "../../include/uav_ugv_collaboration_module/utils.h"

struct PclPoint{

    PclPoint(){
        xyz_.setZero();
        color_.setZero();
    }

    Vector3d xyz_;
    Vector3d color_;
};

class EnvironmentRepresentation{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EnvironmentRepresentation(const std::string& cloudName);
        ~EnvironmentRepresentation(){}

        void loadFromPCLcloud(const std::shared_ptr<open3d::PointCloud> pointCloud,
                              const float &square_size,
                              const Vector2d& imgCenter = Vector2d(0.f, 0.f),
                              const float& radius = 0.01,
                              const cv::Size& gridMapSize = cv::Size(1300,1300) );

        void computeMMGridMap();

        // GetFunctions()
        inline cv::Mat getRgbImg(){ return rgbImg; }
        inline cv::Mat getExgColorImg(){ return exgImgColor; };
        inline cv::Mat getXyzImgUchar(){ return xyzImgUChar; };
        inline cv::Mat getXyzImg(){ return xyzImg; };
        inline cv::Mat getelevImg(){ return xyzImg; };
        inline cv::Mat getExgImg(){ return exgImg; };

    private:

        PclPoint computeAveragePoint(std::vector<PclPoint> &ptVec,
                                     const unsigned int& col,
                                     const unsigned int& row);

        std::vector< std::vector<PclPoint> > _gridMap;
        const std::string _cloudName;
        int _width, _height = 0;
        Vector3d minPt, maxPt;
        float _square_size, x_coord, y_coord;
        cv::Mat exgImg, elevImg, xyzImg, xyzImgUChar, exgImgColor, rgbImg;
        int altitude_scale;
};

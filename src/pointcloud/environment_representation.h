#pragma once
#include "../../include/uav_ugv_collaboration_module/utils.h"
 

struct gridPoint{

    PCLptXYZRGB* pt;
    PCLptXYZRGB_Vector pt_vector;
    int ExG;
    float relative_height;
};

class EnvironmentRepresentation{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EnvironmentRepresentation(const std::string& cloudName);
        ~EnvironmentRepresentation(){}

        void loadFromPCLcloud(const PCLPointCloudXYZRGB::Ptr& pointCloud , const float &square_size);
        void computeMMGridMap();
        PCLptXYZRGB computeAveragePoint(std::vector<PCLptXYZRGB> &ptVec,
                                        const unsigned int& col,
                                        const unsigned int& row);

        /*void computeFilteredPcl(const Vector3i& color);
        void loadFromPcl( const PCLPointCloudXYZRGB::Ptr& pointCloud );
        void copyFrom(const PCLptXYZRGB_Vector& pcl_data,
                      const Vector3d& t,
                      const Vector3& q);
                      
        // Get() functions 
        int inline getSize(){ return _PointCloud.size(); }
        int inline getFilteredSize(){ return _PointCloudFiltered.size(); }
        Vector3d inline getInitGuessT(){return _init_guess_t;}
        Vector3 inline getInitGuessQ(){return _init_guess_q;}
        PCLptXYZRGB inline getPointCloudAt(const int& i){return _PointCloud[i];}
        bool getPointCloudFilteredAtWithRange(const int& i, const float& range, PCLptXYZRGB& pt );
        PCLptXYZRGB inline getPointCloudFilteredAt(const int& i){return _PointCloudFiltered[i];}
        PCLptXYZRGB_Vector inline getPointCloud(){ return _PointCloud;}
        PCLptXYZRGB_Vector inline getPointCloudFiltered(){ return _PointCloudFiltered;}
        cv::Mat inline getRGBImg(){ return _RGBImg;}
        cv::Mat inline getXYZImg(){ return _XYZImg;}
        cv::Mat inline getExGImg(){ return _ExGImg;}
        Vector2 inline getScaleNoise(){ return _scaleNoise;}
        float inline getYawNoise(){ return _YawNoise;}
        Vector2d inline getTranslNoise(){ return _TranslNoise;}

        // Set() functions
        void inline setInitGuessT( const Vector3d& init_guess_t ){_init_guess_t = init_guess_t;};
        void inline setInitGuessQ( const Vector3& init_guess_q ){_init_guess_q = init_guess_q;};

        // Other functions
        void BrightnessEnhancement(const int& brightness, const bool& convertToBW = false);
        void affineTransformPointCloud( const Eigen::Matrix3f& R, const Eigen::Vector3f& t );
        void scalePointCloud(const Vector2& scale_factors);
        void transformPointCloud(const Transform& tf);
        void computeDesifiedPCL(const cv::Size& outp_img_size, const Vector2& offset,
                                const std::string& package_path, const Vector3i& color, const bool& take_higher = false);
        void downsamplePointCloud(const float& downsampl_range);
        void addNoise(const float& scaleMag, const float& TranslMag, const float& YawMag);
        void planeNormalization();*/

    private:

        std::vector< std::vector<PCLptXYZRGB> > _gridMap;
        const std::string _cloudName;
        int _width, _height = 0;
        PCLptXYZRGB minPt, maxPt;
        float _square_size, x_coord, y_coord;
        cv::Mat exgImg, elevImg, xyzImg, xyzImgUChar, exgImgColor;
        int altitude_scale;

        /*PCLptXYZRGB computeAveragePoint(const std::vector<int>& Idx, const std::vector<float>& Radius, const float &range);
        void computeImgs(const Vector2& offset, const Vector3i& color, const bool& take_higher);
        void computePlanarKDTree();

        // KDTree Variables
        PCLKDtreeXYZ planar_kdtree;

        // Cloud Data
        const std::string _cloudName; 
        PointData_Vector _PointCloudData;
        PCLptXYZRGB_Vector _PointCloud;
        PCLptXYZRGB_Vector _PointCloudFiltered;

        // Initial Guesses
        Vector3d _init_guess_t;
        Vector3 _init_guess_q;
        Vector2 _scaleNoise;
        Vector2d _TranslNoise;
        float _YawNoise;

        // Cloud Organized Data
        cv::Mat _ExGImg, _ElevImg, _XYZImg, _RGBImg, _ExGColorImg;*/
};

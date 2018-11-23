#pragma once
#include "../../include/uav_ugv_collaboration_module/utils.h"
 
class PointCloud{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        PointCloud(const std::string& cloudName);
        ~PointCloud(){}

        void computeFilteredPcl(const Vector3i& color);
        void loadFromPcl( const PCLPointCloud::Ptr& pointCloud );
        void copyFrom(const std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& pcl_data, 
                      const Vector3d& t,
                      const Vector3& q);
                      
        // Get() functions 
        int inline getSize(){ return _PointCloud.size(); }
        int inline getFilteredSize(){ return _PointCloudFiltered.size(); }
        Vector3d inline getInitGuessT(){return _init_guess_t;}
        Vector3 inline getInitGuessQ(){return _init_guess_q;}
        pcl::PointXYZRGB inline getPointCloudAt(const int& i){return _PointCloud[i];}
        bool getPointCloudFilteredAtWithRange(const int& i, const float& range, pcl::PointXYZRGB& pt );
        pcl::PointXYZRGB inline getPointCloudFilteredAt(const int& i){return _PointCloudFiltered[i];}
        std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > inline getPointCloud(){ return _PointCloud;}
        std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > inline getPointCloudFiltered(){ return _PointCloudFiltered;}
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

    private:

        pcl::PointXYZRGB computeAveragePoint(const std::vector<int>& Idx, const std::vector<float>& Radius, const float &range);
        void computeImgs(const Vector2& offset, const Vector3i& color, const bool& take_higher);
        void computePlanarKDTree();

        // KDTree Variables
	    pcl::KdTreeFLANN<pcl::PointXYZ> planar_kdtree;

        // Cloud Data
        const std::string _cloudName; 
        std::vector< _PointData, Eigen::aligned_allocator<_PointData> > _PointCloudData;
        std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > _PointCloud;
        std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > _PointCloudFiltered;

        // Initial Guesses
        Vector3d _init_guess_t;
        Vector3 _init_guess_q;
        Vector2 _scaleNoise;
        Vector2d _TranslNoise;
        float _YawNoise;

        // Cloud Organized Data
        cv::Mat _ExGImg, _ElevImg, _XYZImg, _RGBImg, _ExGColorImg;
};

#pragma once
#include "pointcloud_handler.h"

void onMouse(int evt, int x, int y, int flags, void* param) {

    cv::Point* ptPtr = (cv::Point*)param;

    if(evt == CV_EVENT_LBUTTONDOWN) {
        ptPtr->x = x;
        ptPtr->y = y;
    } else if (evt == CV_EVENT_MBUTTONDOWN) {
        ptPtr->x = -1;
        ptPtr->y = -1;
    }
}

class GroundTruthEvaluation : public PointCloudHandler{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        GroundTruthEvaluation();
        ~GroundTruthEvaluation(){
            std::cerr << FRED("[GroundTruthEvaluation] deleting\n");
            std::cerr << BOLD( FRED("[GroundTruthEvaluation] deleted\n") );
        }

        // Set() functions
        void Match(const std::string& cloud1_name, const std::string& cloud2_name, const cv::Size& size , const Eigen::Vector2f &scale);
        void computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key, 
                                                 const std::string& moving_cloud_key); 

        void computeDensifiedPCLs(const std::string& fixed_cloud,
                                   const std::string& moving_cloud,
                                   const cv::Size& outp_img_size);

        void showCloud(const std::string& cloud_to_show); 
        void spingUntilDeath();   
        void setAffineTransform(const Eigen::Matrix3f& R, const Eigen::Vector3f& t);
        void writeAffineTransform(const Eigen::Vector2f &scale);
        const Eigen::Matrix4f getGTtransform(){ return _T; };

    private:

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        Eigen::Matrix4f _T;
};

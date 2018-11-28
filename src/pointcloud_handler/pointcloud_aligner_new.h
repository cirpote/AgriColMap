#pragma once
#include "pointcloud_handler.h"

class PointCloudAlignerNew : public PointCloudHandler{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        PointCloudAlignerNew();
        ~PointCloudAlignerNew(){
            std::cerr << FRED("[PointCloudAligner] deleting\n");
            std::cerr << BOLD( FRED("[PointCloudAligner] deleted\n") );
        }

        // Set() functions
        /*void Match(const std::string& cloud1_name, const std::string& cloud2_name, const Eigen::Vector2f& scale, const string& iter_num, const cv::Size& size = cv::Size(1000,1000));
        void MatchCPD(const std::string& cloud1_name, const std::string& cloud2_name, const cv::Size& size, const Eigen::Vector2f& scale, const string& iter_num);
        void MatchGoICP(const std::string& cloud1_name, const std::string& cloud2_name);*/
        void computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key,
                                                 const std::string& moving_cloud_key); 
        void addNoise(const std::string& cloud_key,
                      const float& scaleMag,
                      const float& TranslMag,
                      const float& YawMag);

        /*void computeDensifiedPCLs(const std::string& fixed_cloud,
                                   const std::string& moving_cloud,
                                   const cv::Size& outp_img_size = cv::Size(1000,1000));

        void writeAffineTransform(const string& iter, const string &cloud);
        void writeAffineTransformCPD(const string& iter, const string& cloud);*/


    private:

        /*FImage computeSiftCorrespondeces( const std::string& cloud1_name, const std::string& cloud2_name, const MatchingType& type);
        void WriteDenseOpticalFlow(const int& w, const int& h, const string& cloud, const string& iter);
        void showDOFCorrespondeces(const int& len, const string &cloud1_name, const string &cloud2_name, const cv::Size& size);
        void computeAndApplyDOFTransform(const std::string& cloud1_name, const std::string& cloud2_name, int &len);
        void downsamplePointClouds(const std::string& cloud1_name, const std::string& cloud2_name);
        void finalRefinement(const std::string& cloud1_name, const std::string& cloud2_name, const Eigen::Vector2f &scale);*/

        FImage img1, img1Cloud, img2, img2Cloud, matches, filteredMatches;

        // Dense Optical Flow Estimation
        CPM cpm;

        // Final Affine Transform+
        Matrix3 _R;
        Vector3 _t;

        vector<Vector3> fix_pts, mov_pts;
};

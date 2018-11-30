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
        void computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key,
                                                 const std::string& moving_cloud_key); 
        void addNoise(const std::string& cloud_key,
                      const float& scaleMag,
                      const float& TranslMag,
                      const float& YawMag);

        void computeExGFilteredPointClouds(const std::string& mov_cloud_key, const std::string& fix_cloud_key);
        void computeEnvironmentalModels(const std::string& mov_cloud_key, const std::string& fix_cloud_key);
        void Match(const std::string& cloud1_name, const std::string& cloud2_name, const Eigen::Vector2f& scale, const string& iter_num, const cv::Size& size = cv::Size(1000,1000));

        std::unordered_map<std::string, const boost::shared_ptr<EnvironmentRepresentation> > ERMap;

    private:

        void showDOFCorrespondeces(const int& len, const string &cloud1_name, const string &cloud2_name, const cv::Size& size);
        void writeAffineTransform(const string& iter, const string &cloud);
        void WriteDenseOpticalFlow(const int& w, const int& h, const string& cloud, const string& iter);
        void computeAndApplyDOFTransform(const std::string& cloud1_name, const std::string& cloud2_name, int &len);

        void downsamplePointClouds(const std::string& cloud1_name, const std::string& cloud2_name);
        void finalRefinement(const std::string& cloud1_name, const std::string& cloud2_name);

        FImage img1, img1Cloud, img2, img2Cloud, matches, filteredMatches;

        // Dense Optical Flow Estimation
        CPM cpm;

        // Final Affine Transform+
        Matrix3 _R;
        Vector3 _t;

        vector<Vector3> fix_pts, mov_pts;
};

#pragma once
#include "pointcloud_handler.h"

class PointCloudAligner : public PointCloudHandler{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PointCloudAligner();
        ~PointCloudAligner(){
            std::cerr << FRED("[PointCloudAligner] deleting\n");
            std::cerr << BOLD( FRED("[PointCloudAligner] deleted\n") );
        }

        // Get() functions
        std::shared_ptr<open3d::PointCloud> inline getPcl(const std::string& cloud_to_get){ return pclMap[cloud_to_get]; }
        std::shared_ptr<open3d::PointCloud> inline getFilteredPcl(const std::string& cloud_to_get){ return pclMapFiltered[cloud_to_get]; }

        void GroundTruthTransformPointCloud(const std::string& cloud_key);

        void computeExGFilteredPointCloud(const string& cloud_key,
                                           const Vector3i& cloud_color);

        // Set() functions
        void computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key,
                                                 const std::string& moving_cloud_key);
        void addNoise(const std::string& cloud_key,
                      const float& scaleMag,
                      const float& TranslMag,
                      const float& YawMag);


        void computeEnvironmentalModels(const std::string& mov_cloud_key,
                                        const std::string& fix_cloud_key);


        void Match(const std::string& cloud1_name,
                   const std::string& cloud2_name,
                   const Eigen::Vector2f& scale,
                   const string& iter_num,
                   const cv::Size& size = cv::Size(1000,1000));

        std::unordered_map<std::string, const boost::shared_ptr<EnvironmentRepresentation> > ERMap;

    private:

        void showDOFCorrespondeces(const int& len,
                                   const string &cloud1_name,
                                   const string &cloud2_name,
                                   const cv::Size& size);

        void writeAffineTransform(const string& iter,
                                  const string &cloud);

        void WriteDenseOpticalFlow(const int& w,
                                   const int& h,
                                   const string& cloud,
                                   const string& iter);

        void computeAndApplyDOFTransform(const std::string& cloud1_name,
                                         const std::string& cloud2_name,
                                         int &len);

        void finalRefinement(const std::string& cloud1_name,
                             const std::string& cloud2_name);

        FImage img1, img1Cloud, img2, img2Cloud, matches, filteredMatches;

        // Dense Optical Flow Estimation
        CPM cpm;

        vector<Vector3d> fix_pts, mov_pts;

        // Final Affine Transform+
        Matrix3d _R;
        Vector3d _t;
};

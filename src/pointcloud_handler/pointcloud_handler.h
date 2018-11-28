#pragma once
#include "../pointcloud/pointcloud.h"
#include "../pointcloud/environment_representation.h"

using namespace std;

class PointCloudHandler{

    public:

        typedef std::unordered_map<std::string, const boost::shared_ptr<PointCloud> > PointCloudUnorderedMap;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        PointCloudHandler() : _package_path( ros::package::getPath("uav_ugv_collaboration_module") ), _scaleNoise(1.f, 1.f){
            std::cerr << std::fixed << std::setprecision(6); }
        ~PointCloudHandler(){}

        void loadCloud(const std::string& cloud_name, const std::string& cloud_path, const std::string& cloud_key);
        void initFromYaml(const std::string& yaml_file);
        void loadFromDisk(const std::string& fixed_cloud_key, const std::string& moving_cloud_key);
        void loadFixedCloudFromDisk(const std::string& cloud_name, const std::string& cloud_path, const std::string& cloud_key);
        void loadMovingCloudFromDisk(const std::string& cloud_name, const std::string& cloud_path, const std::string& cloud_key, const string &fixed_cloud_key, const Vector2& scale);
        void cropFixedPointCloud(const std::string& fix_crop_cloud, const string &mov_crop_cloud, const string &fix_cloud);

        void scalePointCloud( const Vector2& scale_factors,
                              const std::string& cloud_to_scale );
        void affineTransformPointCloud(const Eigen::Matrix3f& R,
                                       const Eigen::Vector3f& t,
                                       const std::string& cloud_to_transform){ _pclMap[cloud_to_transform]->affineTransformPointCloud(R, t); }
        void GroundTruthTransformPointCloud(const std::string& cloud_to_transform){
            _pclMap[cloud_to_transform]->affineTransformPointCloud(_GTMap[cloud_to_transform]->_Rgt, _GTMap[cloud_to_transform]->_tgt);}
        void inline downsamplePointCloud( const float& downsampl_range,
                                          const std::string& cloud_to_downsample ) { _pclMap[cloud_to_downsample]->downsamplePointCloud(downsampl_range); }


        // Set Functions
        void inline setInitMovScale(const Vector2& mov_scale){ _init_mov_scale = mov_scale; }

        // Get Functions
        std::string inline getPackagePath(){ return _package_path; }
        std::string inline getMovingCloudPath(){ return _moving_pcl_path;}
        PCLPointCloudXYZRGB::Ptr inline getPointCloud(const std::string& cloud_name){ return pclMap[cloud_name]; }
        boost::shared_ptr<GroundTruth> inline getGroundTruth(const std::string& cloud_name){ return _GTMap[cloud_name]; }
        bool inline getVerbosityLevel(){return _verbosity;}
        const Vector2 inline getInitMovScale(){return _init_mov_scale;}

        // New variables
        std::unordered_map< std::string, PCLPointCloudXYZRGB::Ptr> pclMap;
        std::unordered_map< std::string, Vector3d> initGuessTMap;
        std::unordered_map< std::string, Vector3> initGuessQMap;
        std::unordered_map< std::string, const boost::shared_ptr<Transform> > GTtfMap;
        Vector2 _scaleNoise;
        Vector2d _TranslNoise;
        float _YawNoise;

        // New functions
        void planeNormalization(const std::string& cloud_key);

    protected:

        // External Params
        bool _storeDenseOptFlw = false;
        bool _showDOFCorrespondences = false;
        int _max_iter_num = 1;
        bool _verbosity = false;
        int _dense_optical_flow_step = 5;
        float _downsampling_rate = 0.15;
        float _search_radius = 0.15;
        bool _useVisualFeatures = true;
        bool _useGeometricFeatures = true;

        // Ground Truth & Point Clouds
        PointCloudUnorderedMap _pclMap;
        GroundTruthUnorderedMap _GTMap;
        TransformUnorderedMap _InitTfMap;

        const std::string _package_path;

        //PCLPointCloudXYZRGB::Ptr _pcl_data_moving;
        //Vector3 init_guess_T_mov, init_guess_Q_mov;

        // Strings and types that encode the paths and the extensions for the Point-Clouds to measure
		std::string _fixed_pcl_path, _moving_pcl_path, _fixed_pcl, _moving_pcl;

        // Relative Scale
        Vector2 _init_mov_scale;

};

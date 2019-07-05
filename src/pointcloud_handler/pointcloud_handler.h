#pragma once
#include "../environment_model/environment_representation.h"
#include "../packageDir.h"

using namespace std;

class PointCloudHandler{

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PointCloudHandler() : _scaleNoise(1.f, 1.f){
            std::cerr << std::fixed << std::setprecision(6); }
        ~PointCloudHandler(){}

        void loadCloudTemp(const std::string& cloud_path,  PCLPointCloudXYZRGB::Ptr cloud);

        void loadCloud(const std::string& cloud_name,
                       const std::string& cloud_path,
                       const std::string& cloud_key);

        void initFromYaml(const std::string& yaml_file);

        void loadFromDisk(const std::string& fixed_cloud_key,
                          const std::string& moving_cloud_key);

        void scalePointCloud( const Vector2& scale_factors,
                              const std::string& cloud_to_scale,
                              const std::string& cloud_type );

        void transformPointCloud( const Transform& tf,
                                  const std::string& cloud_to_scale,
                                  const std::string& cloud_type );

        void loadFixedCloudFromDisk(const std::string &cloud_name,
                                    const std::string &cloud_path,
                                    const std::string &cloud_key);

        void loadMovingCloudFromDisk(const std::string &cloud_name,
                                     const std::string &cloud_path,
                                     const std::string &cloud_key,
                                     const std::string &fixed_cloud_key,
                                     const Vector2 &scale);

        void BrightnessEnhancement(const std::string& cloud_key, const int& brightness);

        // Set Functions
        void inline setInitMovScale(const Vector2& mov_scale){ _init_mov_scale = mov_scale; }

        // Get Functions
        std::string inline getPackagePath(){ return _package_path; }
        std::string inline getMovingCloudPath(){ return _moving_pcl_path;}
        PCLPointCloudXYZRGB::Ptr inline getPointCloud(const std::string& cloud_name){ return pclMap[cloud_name]; }
        //boost::shared_ptr<GroundTruth> inline getGroundTruth(const std::string& cloud_name){ return GTMap[cloud_name]; }
        bool inline getVerbosityLevel(){return _verbosity;}
        const Vector2 inline getInitMovScale(){return _init_mov_scale;}

    protected:

        void downsamplePCL(const std::string& cloud_name,
                           const float& rate = 0.f);

        void planeNormalization(const std::string& cloud_key);

        void ExGFilterPCL(const string &cloud_key,
                          const Vector3i& cloud_color);

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
        float _vis_feat_weight, _geom_feat_weight;

        // Algorithm Variables
        PCLXYZRGB_unMap pclMap, pclMapFiltered, pclMapFilteredDownSampled;
        std::unordered_map< std::string, Vector3d> initGuessTMap;
        std::unordered_map< std::string, Vector3> initGuessQMap;
        GroundTruthUnorderedMap GTtfMap;
        TransformUnorderedMap _initTfMap;
        Vector2 _scaleNoise;
        Vector2d _TranslNoise;
        float _YawNoise;


        // Strings and types that encode the paths and the extensions for the Point-Clouds to measure
		std::string _fixed_pcl_path, _moving_pcl_path, _fixed_pcl, _moving_pcl;

        // Relative Scale
        Vector2 _init_mov_scale;

};

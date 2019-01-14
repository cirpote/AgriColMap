#include "pointcloud_handler.h"

PointCloudHandler::PointCloudHandler() : _scaleNoise(1.f, 1.f){
    std::cerr << std::fixed << std::setprecision(6);
    horiz_cloud = std::shared_ptr<open3d::PointCloud>(new open3d::PointCloud());
    horiz_cloud->points_.push_back(Vector3d(1,1,0));
    horiz_cloud->points_.push_back(Vector3d(1,0,0));
    horiz_cloud->points_.push_back(Vector3d(1,-1,0));
    horiz_cloud->points_.push_back(Vector3d(0,1,0));
    horiz_cloud->points_.push_back(Vector3d(0,0,0));
    horiz_cloud->points_.push_back(Vector3d(0,-1,0));
    horiz_cloud->points_.push_back(Vector3d(-1,1,0));
    horiz_cloud->points_.push_back(Vector3d(-1,0,0));
    horiz_cloud->points_.push_back(Vector3d(-1,-1,0));
    horiz_cloud->points_.push_back(Vector3d(2,2,0));
    horiz_cloud->points_.push_back(Vector3d(2,0,0));
    horiz_cloud->points_.push_back(Vector3d(2,-2,0));
    horiz_cloud->points_.push_back(Vector3d(0,2,0));
    horiz_cloud->points_.push_back(Vector3d(0,-2,0));
    horiz_cloud->points_.push_back(Vector3d(-2,2,0));
    horiz_cloud->points_.push_back(Vector3d(-2,0,0));
    horiz_cloud->points_.push_back(Vector3d(-2,-2,0));
}

void PointCloudHandler::loadCloud(const std::string &cloud_name, const std::string &cloud_path, const std::string &cloud_key){

    // Check for the fixed cloud path
    string input_pcl_str = _package_path + "/maps/" + cloud_path + "/" + cloud_name + ".ply";
    ifstream input_pcl( input_pcl_str );
    if(!input_pcl)
        ExitWithErrorMsg("File Does Not Exist: " + input_pcl_str);

    std::shared_ptr<open3d::PointCloud> cloud_ptr ( new open3d::PointCloud() );
    if (!ReadPointCloud(input_pcl_str, *cloud_ptr)) {
        cerr << FGRN("Failed to read: ") << input_pcl_str << "\n";
        return;
    }

    pclMap.emplace( cloud_key, cloud_ptr );
    cerr << FGRN("Correctly Imported: ") << input_pcl_str << " " << pclMap[cloud_key]->points_.size() << " Points" << "\n";

    // Reading the fixed Offset
    string fixed_offset;
    ifstream fixed_offset_file( _package_path + "/maps/" + cloud_path + "/" + "offset.xyz" );
    getline(fixed_offset_file, fixed_offset);
    vector<double> fixed_utm_vec = vectorFromString(fixed_offset);
    initGuessTMap.emplace(cloud_key, Vector3d(fixed_utm_vec[0], fixed_utm_vec[1], fixed_utm_vec[2]));
    initGuessQMap.emplace(cloud_key, Vector3d(fixed_utm_vec[3], fixed_utm_vec[4], fixed_utm_vec[5]));

    if(_verbosity){
        cerr << FYEL("Initial Guess T Correctly Imported: ") << initGuessTMap[cloud_key].transpose() << "\n";
        cerr << FYEL("Initial Guess Q Correctly Imported: ") << initGuessQMap[cloud_key].transpose() << "\n" << "\n";
    }

    Vector3d mean(0.f, 0.f, 0.f);
    for(unsigned int i = 0; i < pclMap[cloud_key]->points_.size(); ++i)
        mean += pclMap[cloud_key]->points_[i];
    mean /= pclMap[cloud_key]->points_.size();

    Matrix4 norm_T = Matrix4::Identity();
    norm_T.block<3, 1>(0, 3) = -mean;
    transformPointCloud(norm_T, cloud_key, "rgb");

}

void PointCloudHandler::planeNormalization(const std::string& cloud_key){

    Matrix4 current_transformation = Matrix4::Identity();
    std::shared_ptr<open3d::PointCloud> downsampled_cloud = open3d::VoxelDownSample(*pclMap[cloud_key], 0.2);

    open3d::EstimateNormals(*downsampled_cloud);
    open3d::EstimateNormals(*horiz_cloud);
    open3d::RegistrationResult result_icp = open3d::RegistrationICP(*downsampled_cloud,
                                                                    *horiz_cloud,
                                                                    1,
                                                                    current_transformation);
    result_icp.transformation_.block<3, 1>(0, 3) = Vector3d(0.f, 0.f, 0.f);

    Vector3d ez_B = result_icp.transformation_.block<3, 1>(0,2);
    ez_B(0) *= -1; ez_B(1) *= -1;

    float yaw_des = initGuessQMap[cloud_key](2)*(3.14/180);
    Vector3d ey_C( -sin(yaw_des), cos(yaw_des), 0 );
    Vector3d ex_B = ey_C.cross( ez_B ) / ( ey_C.cross( ez_B ) ).norm();
    Vector3d ey_B = ez_B.cross( ex_B ) / ( ez_B.cross( ex_B ) ).norm();

    result_icp.transformation_.block<3, 1>(0,0) = ex_B;
    result_icp.transformation_.block<3, 1>(0,1) = ey_B;
    result_icp.transformation_.block<3, 1>(0,2) = ez_B;

    transformPointCloud(result_icp.transformation_.transpose(), cloud_key, "rgb");
    return;
}

void PointCloudHandler::initFromYaml(const std::string& yaml_file){

    cerr << FBLU("Initializing from:") << " " << yaml_file << "\n" << "\n";
    YAML::Node configuration = YAML::LoadFile(yaml_file);

    // Loading info for fixed input cloud from YAML
    _fixed_pcl_path = configuration["input_clouds"]["cloud_fixed_path"].as<string>();
    _fixed_pcl = configuration["input_clouds"]["cloud_fixed_name"].as<string>();

    // Check for the fixed cloud path
    string input_pcl = _package_path + "/maps/" + _fixed_pcl_path + "/" + _fixed_pcl + ".ply";
    ifstream in_fixed_pcl( input_pcl );
    if(!in_fixed_pcl)
        ExitWithErrorMsg("File Does Not Exist: " + input_pcl);

    // Loading info for moving input cloud from YAML
    _moving_pcl_path = configuration["input_clouds"]["cloud_moving_path"].as<string>();
    _moving_pcl = configuration["input_clouds"]["cloud_moving_name"].as<string>();

    // Check for the moving cloud path
    input_pcl = _package_path + "/maps/" + _moving_pcl_path + "/" + _moving_pcl + ".ply";
    ifstream mov_fixed_pcl( input_pcl );
    if(!mov_fixed_pcl)
        ExitWithErrorMsg("File Does Not Exist: " + input_pcl);

    // Reading mov_scale
    vector<float> mov_scale_vec = configuration["input_clouds"]["relative_scale"].as<std::vector<float>>();
    _init_mov_scale << mov_scale_vec[0], mov_scale_vec[1];

    _max_iter_num = configuration["aligner_params"]["max_iter_number"].as<int>();
    _verbosity = configuration["aligner_params"]["verbosity"].as<bool>();
    _storeDenseOptFlw = configuration["aligner_params"]["store_dense_optical_flow"].as<bool>();
    _showDOFCorrespondences = configuration["aligner_params"]["show_dpf_correspondences"].as<bool>();
    _dense_optical_flow_step = configuration["aligner_params"]["dense_optical_flow_step"].as<int>();
    _downsampling_rate = configuration["aligner_params"]["downsampling_rate"].as<float>();
    _search_radius = configuration["aligner_params"]["search_radius"].as<float>();
    _useVisualFeatures = configuration["aligner_params"]["use_visual_features"].as<bool>();
    _vis_feat_weight = configuration["aligner_params"]["visual_features_weight"].as<float>();
    _useGeometricFeatures = configuration["aligner_params"]["use_geometric_features"].as<bool>();
    _geom_feat_weight = configuration["aligner_params"]["geometric_features_weight"].as<float>();

}

void PointCloudHandler::loadFromDisk(const std::string& fixed_cloud_key, const std::string& moving_cloud_key){

    loadCloud(_moving_pcl, _moving_pcl_path, moving_cloud_key);
    loadCloud(_fixed_pcl, _fixed_pcl_path, fixed_cloud_key);

    string ground_truth_tf_path = _package_path + "/params/output/" + _moving_pcl_path + "_AffineGroundTruth.txt";
    ifstream mov_fixed_pcl( ground_truth_tf_path ); bool groundTruth = false;
    if(mov_fixed_pcl) {
        string affine_gt_tf; groundTruth = true;
        getline(mov_fixed_pcl, affine_gt_tf);
        Matrix3d _Rgt; Vector3d _tgt; Vector2d _scl;
        AffineTransformFromString(affine_gt_tf, _Rgt, _tgt, _scl);
        GTtfMap.emplace( moving_cloud_key, boost::shared_ptr<GroundTruth>(new GroundTruth(_Rgt, _tgt, _scl)));
    }

    if(_verbosity && groundTruth){
        cerr << FYEL("Ground Truth Affine Matrix: ") << "\n" << GTtfMap[moving_cloud_key]->_Rgt << "\n";
        cerr << FYEL("Ground Truth Translation: ") << GTtfMap[moving_cloud_key]->_tgt.transpose() << "\n";
        cerr << FYEL("Ground Truth Relative Scale: ") << GTtfMap[moving_cloud_key]->_rel_scl.transpose() << "\n" << "\n";
    }

}

void PointCloudHandler::scalePointCloud(const Vector2 &scale_factors, 
                                        const string &cloud_to_scale,
                                        const std::string& cloud_type){

    Matrix4 scaling_tf = Matrix4::Identity();
    scaling_tf(0,0) *= scale_factors(0);
    scaling_tf(1,1) *= scale_factors(1);

    transformPointCloud(scaling_tf, cloud_to_scale, cloud_type);

    return;
}

void PointCloudHandler::transformPointCloud( const Matrix4& tf,
                                             const std::string& cloud_to_scale,
                                             const std::string& cloud_type){

    if ( cloud_type.compare("rgb") == 0 ) {
        pclMap[cloud_to_scale]->Transform(tf);
    } else if ( cloud_type.compare("exg") == 0 ) {
        pclMapFiltered[cloud_to_scale]->Transform(tf);
    } else if ( cloud_type.compare("exg_downsampled") == 0 ) {
        pclMapFilteredDownSampled[cloud_to_scale]->Transform(tf);
    }

    return;
}


void PointCloudHandler::ExGFilterPCL(const string &cloud_key, const Vector3i& cloud_color){

    std::shared_ptr<open3d::PointCloud> data_filtered( new open3d::PointCloud() );
    unsigned int iter = 0;
    for(Vector3d pt : pclMap[cloud_key]->colors_){
        if( (float) computeExGforXYZRGBPoint(pt) > 30){
            pt(0) = cloud_color(0); pt(1) = cloud_color(1); pt(2) = cloud_color(2);
            data_filtered->points_.push_back(pclMap[cloud_key]->points_[iter]);
            data_filtered->colors_.push_back(pt);
        }
        iter++;
    }
    pclMapFiltered.emplace( cloud_key, data_filtered );
    return;
}

void PointCloudHandler::downsamplePCL(const std::string& cloud_name, const float& rate){


    float down_rate;
    if( rate == 0.f )
        down_rate = _downsampling_rate;
    else
        down_rate = rate;


    // Filtering the PCL cloud PointCloud
    std::shared_ptr<open3d::PointCloud> _pcl_filtered = open3d::VoxelDownSample(*pclMapFiltered[cloud_name], down_rate);
    pclMapFilteredDownSampled.emplace( cloud_name, _pcl_filtered);


    cerr << "\n";
    cerr << FBLU("Downsampling " + cloud_name + " Cloud... ") << "\n";
    int cloud1_size = pclMapFiltered[cloud_name]->points_.size();
    cerr << FGRN("Cloud DownSampled: ") << cloud1_size << FGRN(" ==> ") <<
            pclMapFilteredDownSampled[cloud_name]->points_.size() << " DownSampling Factor: " << down_rate << "\n" << "\n";
}


void PointCloudHandler::loadFixedCloudFromDisk(const std::string &cloud_name,
                                               const std::string &cloud_path,
                                               const std::string &cloud_key){

    loadCloud(cloud_name, cloud_path, cloud_key);
    planeNormalization(cloud_key);
}

void PointCloudHandler::loadMovingCloudFromDisk(const std::string &cloud_name,
                                                const std::string &cloud_path,
                                                const std::string &cloud_key,
                                                const std::string &fixed_cloud_key,
                                                const Vector2 &scale){

    loadCloud(cloud_name, cloud_path, cloud_key);
    planeNormalization(cloud_key);

    // Normalized along X and Y axis the Fixed Cloud
    Vector3d diff_t( initGuessTMap[cloud_key] - initGuessTMap[fixed_cloud_key] );
    _initTfMap.emplace(cloud_key, boost::shared_ptr<Transform>(new Transform(Transform::Identity())) );
    _initTfMap[cloud_key]->translation() << diff_t.cast<double>();

    cerr << FBLU("InitMovScale Set to: ") << scale.transpose() << "\n";
    scalePointCloud( scale, cloud_key, "rgb");

    string ground_truth_tf_path = _package_path + "/params/output/" + cloud_path + "_AffineGroundTruth.txt";
    ifstream ground_truth( ground_truth_tf_path ); bool groundTruth = false;
    if(ground_truth) {
        string affine_gt_tf; groundTruth = true;
        getline(ground_truth, affine_gt_tf);
        Matrix3d _Rgt; Vector3d _tgt; Vector2d _scl;
        AffineTransformFromString(affine_gt_tf, _Rgt, _tgt, _scl);
        GTtfMap.emplace( cloud_key, boost::shared_ptr<GroundTruth>(new GroundTruth(_Rgt, _tgt, _scl)));
    } else {
        ExitWithErrorMsg("File Does Not Exist: " + ground_truth_tf_path);
    }

}

/*void PointCloudHandler::BrightnessEnhancement(const std::string& cloud_key, const int& brightness){

    for( PCLptXYZRGB& pt : pclMap[cloud_key]->points ){
        pt.r = cv::saturate_cast<uchar>( pt.r + (uchar)brightness );
        pt.g = cv::saturate_cast<uchar>( pt.g + (uchar)brightness );
        pt.b = cv::saturate_cast<uchar>( pt.b + (uchar)brightness );
    }
}*/

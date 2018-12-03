#include "pointcloud_aligner_new.h"

using namespace std;

PointCloudAlignerNew::PointCloudAlignerNew() : _R(Matrix3::Identity()),
                                               _t(Vector3::Zero()),
                                               cpm(_vis_feat_weight, _geom_feat_weight) {}

void PointCloudAlignerNew::computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key,
                                                            const std::string& moving_cloud_key){

        if( getVerbosityLevel() )
            cerr << FBLU("Compute And Applying The Relative Initial Guess!") << "\n";

        planeNormalization(fixed_cloud_key);
        planeNormalization(moving_cloud_key);

        // Normalized along X and Y axis the Fixed Cloud
        Vector3d diff_t( initGuessTMap[moving_cloud_key] - initGuessTMap[fixed_cloud_key] );
                    //getPointCloud(moving_cloud_key)->getInitGuessT() - getPointCloud(fixed_cloud_key)->getInitGuessT() );
        _initTfMap.emplace(moving_cloud_key, boost::shared_ptr<Transform>(new Transform(Transform::Identity())) );
        _initTfMap[moving_cloud_key]->translation() << diff_t.cast<float>();

        _init_mov_scale(0) *= _scaleNoise(0);
        _init_mov_scale(1) *= _scaleNoise(1);
        cerr << FBLU("InitMovScale Set to: ") << _init_mov_scale.transpose() << "\n";
        scalePointCloud( _init_mov_scale, moving_cloud_key);

        if( getVerbosityLevel() ){
            cerr << FYEL("Fixed_Cloud Rot_z amount: ") << initGuessQMap[fixed_cloud_key](2) << "\n";
            cerr << FYEL("Moving_Cloud Rot_z amount: ") << initGuessQMap[moving_cloud_key](2) << "\n";
            cerr << FYEL("Init alignment guess: ") << _initTfMap[moving_cloud_key]->translation().transpose() << "\n" << "\n";
        }

}       

void PointCloudAlignerNew::computeExGFilteredPointClouds(const string &mov_cloud_key, const Vector3i& mov_cloud_color,
                                                         const string &fix_cloud_key, const Vector3i& fix_cloud_color){

    PCLPointCloudXYZRGB::Ptr _mov_data_filtered( new PCLPointCloudXYZRGB() );
    for(PCLptXYZRGB pt : pclMap[mov_cloud_key]->points){
        if( (float) computeExGforXYZRGBPoint(pt) > 30){
            pt.r = mov_cloud_color(0); pt.g = mov_cloud_color(1); pt.b = mov_cloud_color(2);
            _mov_data_filtered->points.push_back(pt);
        }
    }
    pclMapFiltered.emplace( mov_cloud_key, _mov_data_filtered );


    PCLPointCloudXYZRGB::Ptr _fix_data_filtered( new PCLPointCloudXYZRGB() );
    for(PCLptXYZRGB pt : pclMap[fix_cloud_key]->points){
        if( (float) computeExGforXYZRGBPoint(pt) > 30){
            pt.r = fix_cloud_color(0); pt.g = fix_cloud_color(1); pt.b = fix_cloud_color(2);
            _fix_data_filtered->points.push_back(pt);
        }
    }
    pclMapFiltered.emplace( fix_cloud_key, _fix_data_filtered );

    return;
}

void PointCloudAlignerNew::computeEnvironmentalModels(const string &mov_cloud_key, const string &fix_cloud_key){

    ERMap.emplace( mov_cloud_key, boost::shared_ptr<EnvironmentRepresentation> ( new EnvironmentRepresentation(mov_cloud_key) ) );
    ERMap[mov_cloud_key]->loadFromPCLcloud( pclMap[mov_cloud_key], 0.02 );
    ERMap[mov_cloud_key]->computeMMGridMap();

    ERMap.emplace( fix_cloud_key, boost::shared_ptr<EnvironmentRepresentation> ( new EnvironmentRepresentation(fix_cloud_key) ) );
    ERMap[fix_cloud_key]->loadFromPCLcloud( pclMap[fix_cloud_key], 0.02, _initTfMap[mov_cloud_key]->translation().head(2) );
    ERMap[fix_cloud_key]->computeMMGridMap();

    return;
}

void PointCloudAlignerNew::addNoise(const std::string& cloud_key, const float& scaleMag, const float& TranslMag, const float& YawMag){

    srand (time(NULL));
    Vector2d vd(1.f, 0.f);
    Vector2 v(1.f,0.f);
    float CircleSamplingAngle = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * 6.28;

    // Translational Noise With Sampling over a Fixed Input Magnitude
    _TranslNoise = Eigen::Rotation2Dd( CircleSamplingAngle )*vd;
    _TranslNoise(0) *= TranslMag +  ( .025*TranslMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * TranslMag ) );
    _TranslNoise(1) *= TranslMag +  ( .025*TranslMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * TranslMag ) );
    cerr << FBLU("Translational Noise: ") << _TranslNoise.transpose() <<
            FBLU(" Translational Norm: ") << _TranslNoise.norm() <<
            FBLU(" _init_guess_t: ") << initGuessTMap[cloud_key].head(2).transpose() <<  //_init_guess_t.head(2).transpose() <<
            FBLU(" _init_guess_t + Noise: ")  <<  (initGuessTMap[cloud_key].head(2) + _TranslNoise).transpose() << "\n"; //(_init_guess_t.head(2) + _TranslNoise).transpose() << "\n";
    initGuessTMap[cloud_key](0) += _TranslNoise(0);
    initGuessTMap[cloud_key](1) += _TranslNoise(1);

    // Translational Noise With Sampling over a Fixed Input Magnitude
    _YawNoise = YawMag + ( .025*YawMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * YawMag ) );
    cerr << FBLU("Yaw Rotational Noise: ") << _YawNoise << "\n";
    //_init_guess_q(2) += _YawNoise;
    initGuessQMap[cloud_key](2) += _YawNoise;

    // Translational Noise With Sampling over a Fixed Input Magnitude
    Vector2 vS = Eigen::Rotation2Df( CircleSamplingAngle )*v;
    vS(0) *= scaleMag + (.025*scaleMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * scaleMag ));
    vS(1) *= scaleMag + (.025*scaleMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * scaleMag ));
    _scaleNoise += vS;

    cerr << FBLU(" Scale Noise: ") << vS.transpose() << FBLU(" Scale Norm: ") << vS.norm() << "\n";
}

void PointCloudAlignerNew::Match( const std::string& cloud1_name, const std::string& cloud2_name,
                               const Eigen::Vector2f& scale, const string& iter_num, const cv::Size& size ){

    cpm.SetParams(_dense_optical_flow_step, _useVisualFeatures, _useGeometricFeatures);
    img1.imcopy( ERMap[cloud1_name]->getExgImg() );
    img2.imcopy( ERMap[cloud2_name]->getExgImg() );
    img1Cloud.imcopy( ERMap[cloud1_name]->getXyzImg() );
    img2Cloud.imcopy( ERMap[cloud2_name]->getXyzImg() );

    cpm.Matching(img1, img1Cloud, img2, img2Cloud, matches);

    if( _storeDenseOptFlw )
        WriteDenseOpticalFlow(img1.width(), img1.height(), cloud2_name, iter_num);

    cpm.VotingScheme(matches, filteredMatches, ERMap[cloud1_name]->getRgbImg(), ERMap[cloud2_name]->getRgbImg());

    cerr << "Total correspondences: " << matches.height() << " Outliers: " << matches.height() - filteredMatches.height() <<
            " Inliers: " << filteredMatches.height() << "\n";

    int len = filteredMatches.height();
    if(len < 10){
        writeAffineTransform(iter_num, cloud2_name);
        ExitWithErrorMsg("too few correspondences");
    }

    if( _showDOFCorrespondences )
        showDOFCorrespondeces(len, cloud1_name, cloud2_name, size);

    computeAndApplyDOFTransform(cloud1_name, cloud2_name, len);
    downsamplePointClouds(cloud1_name, cloud2_name);

    auto compute_start = std::chrono::high_resolution_clock::now();
    finalRefinement(cloud1_name, cloud2_name);
    auto compute_end = std::chrono::high_resolution_clock::now();
    double compute_time = std::chrono::duration_cast<std::chrono::milliseconds>(compute_end - compute_start).count();
    if( getVerbosityLevel() ){
        cerr << "\n";
        std::cerr << FYEL("[SOLVER][compute]: Time Elapsed for finding a solution: ") << compute_time << " milliseconds" << std::endl;
        std::cerr << FBLU("Final Affine Matrix: ") << "\n" << _R << "\n";
        std::cerr << FBLU("Final Translation: ") << _t.transpose() << "\n";
        std::cerr << FBLU("Initial Scale: ") << scale.transpose() << "\n";
    }
    writeAffineTransform(iter_num, cloud2_name);
}

void PointCloudAlignerNew::showDOFCorrespondeces(const int& len, const std::string& cloud1_name, const std::string& cloud2_name, const cv::Size& size){

    cv::Size draw_img_size( size.width*2, size.height );
    cv::Mat drawImg( draw_img_size, CV_8UC3, cv::Scalar(0,0,0) );
    ERMap[cloud1_name]->getRgbImg().copyTo( drawImg( cv::Rect(0, 0, size.width, size.height) )  );
    ERMap[cloud2_name]->getRgbImg().copyTo( drawImg( cv::Rect(size.width, 0, size.width, size.height) )  );

    for( unsigned int i = 0; i < len; ++i )
            cv::line( drawImg,
                      cv::Point(filteredMatches[4*i + 0], filteredMatches[4*i + 1]),
                      cv::Point(filteredMatches[4*i + 2]+size.width, filteredMatches[4*i + 3]),
                      cv::Scalar(0, 255, 0));

    cv::Mat drawImgRes;
    cv::resize(drawImg, drawImgRes, cv::Size( 2000, 1000 ) );

    cv::imshow("matches", drawImgRes);
    cv::waitKey(0);
    cv::destroyWindow("matches");

}

void PointCloudAlignerNew::WriteDenseOpticalFlow(const int& w, const int& h, const string& cloud, const string& iter){

    FImage u, v;
    cpm.Match2Flow(matches, u, v, w, h);
    std::string img_path = getPackagePath() + "/params/output/dense_optical_flow/" + getMovingCloudPath() + iter + "_"
                           + to_string( (_scaleNoise-Vector2(1,1)).norm() ) + "_" +
                           to_string( _TranslNoise.norm() ) + "_" + to_string( _YawNoise ) + ".png";
    OpticFlowIO::SaveFlowAsImage(img_path.c_str(), u.pData, v.pData, w, h);
}


void PointCloudAlignerNew::writeAffineTransform(const string& iter, const string& cloud){

    Vector2 scale = getInitMovScale();
    ofstream outputAffineTf;
    outputAffineTf.open (getPackagePath() + "/params/output/results/" + getMovingCloudPath() +
                         "_AffineGroundTruth_" + iter + "_" + to_string( (_scaleNoise-Vector2(1,1)).norm() ) + "_" +
                         to_string( _TranslNoise.norm() ) + "_" + to_string( _YawNoise ) + ".txt");
    outputAffineTf << _R(0,0) << " " << _R(0,1) << " " << _R(0,2) << " " << _t(0) << " "
                   << _R(1,0) << " " << _R(1,1) << " " << _R(1,2) << " " << _t(1) << " "
                   << _R(2,0) << " " << _R(2,1) << " " << _R(2,2) << " " << _t(2) << " "
                   << scale(0) << " " << scale(1) << " " << _scaleNoise(0) << " "
                   << _scaleNoise(1) << " " << _TranslNoise(0) << " "
                   << _TranslNoise(1) << " " << _YawNoise;
    outputAffineTf.close();
    cerr << FBLU("Ground Truth Affine Transform Written in: ") << getPackagePath() + "/params/output/" +
                 getMovingCloudPath() + "/" + getMovingCloudPath() + "_AffineGroundTruth_" +
                 iter + "_" + to_string( _scaleNoise.norm() ) + "_" + to_string( _TranslNoise.norm() ) +
                 "_" + to_string( _YawNoise) + ".txt" << "\n";
}


void PointCloudAlignerNew::computeAndApplyDOFTransform(const std::string& cloud1_name, const std::string& cloud2_name, int& len){

    cv::Mat fixedXYZImg = ERMap[cloud1_name]->getXyzImg();
    cv::Mat movingXYZImg = ERMap[cloud2_name]->getXyzImg();

    cpd::Matrix fixed(len,3);
    cpd::Matrix moving(len,3);

    int counter = 0;
    while(true){

        if(counter == len)
            break;

        mov_pts.push_back( Vector3 ( movingXYZImg.at<cv::Vec3f>(filteredMatches[4*counter + 3], filteredMatches[4*counter + 2])[0],
                                     movingXYZImg.at<cv::Vec3f>(filteredMatches[4*counter + 3], filteredMatches[4*counter + 2])[1],
                                     movingXYZImg.at<cv::Vec3f>(filteredMatches[4*counter + 3], filteredMatches[4*counter + 2])[2]) );

        fix_pts.push_back( Vector3 ( fixedXYZImg.at<cv::Vec3f>(filteredMatches[4*counter + 1], filteredMatches[4*counter + 0])[0],
                                     fixedXYZImg.at<cv::Vec3f>(filteredMatches[4*counter + 1], filteredMatches[4*counter + 0])[1],
                                     fixedXYZImg.at<cv::Vec3f>(filteredMatches[4*counter + 1], filteredMatches[4*counter + 0])[2] ) );

        fixed(counter,0) = fix_pts[counter](0); fixed(counter,1) = fix_pts[counter](1); fixed(counter,2) = fix_pts[counter](2);
        moving(counter,0) = mov_pts[counter](0); moving(counter,1) = mov_pts[counter](1); moving(counter,2) = mov_pts[counter](2);

        counter++;
    }

    cerr << "\n";
    cerr << FBLU("Computing Initial Alignment From Dense Optical Flow Correspondeces...") << "\n";
    cpd::AffineResult result = cpd::affine(fixed, moving);
    cerr << FGRN("Initial Alignment  Affine Matrix") << "\n";
    cerr << result.transform << "\n";
    cerr << FGRN("Initial Alignment Translation Vector") << "\n";
    cerr << result.translation.transpose() << "\n" << "\n";

    Eigen::Matrix3f R = result.transform.cast <float> ();
    Eigen::Vector3f t = result.translation.cast <float> ();

    _R = R; _t = t;
    Transform LDOF_tf = Transform::Identity();
    LDOF_tf.translation() = _t;
    LDOF_tf.linear() = _R;
    pcl::transformPointCloud(*pclMap[cloud2_name], *pclMap[cloud2_name], LDOF_tf);
    pcl::transformPointCloud(*pclMapFiltered[cloud2_name], *pclMapFiltered[cloud2_name], LDOF_tf);
}

void PointCloudAlignerNew::downsamplePointClouds(const std::string& cloud1_name, const std::string& cloud2_name){

    // Filtering the PCL cloud1 PointCloud
    PCLPointCloudXYZRGB::Ptr _pcl1_filtered( new PCLPointCloudXYZRGB() );
    PCLvoxelGridXYZRGB filter;
    filter.setInputCloud (pclMapFiltered[cloud1_name]);
    filter.setLeafSize (_downsampling_rate, _downsampling_rate, _downsampling_rate);
    filter.filter (*_pcl1_filtered);
    pclMapFilteredDownSampled.emplace( cloud1_name, _pcl1_filtered);

    // Filtering the PCL cloud1 PointCloud
    PCLPointCloudXYZRGB::Ptr _pcl2_filtered( new PCLPointCloudXYZRGB() );
    PCLvoxelGridXYZRGB filter2;
    filter2.setInputCloud (pclMapFiltered[cloud2_name]);
    filter2.setLeafSize (_downsampling_rate, _downsampling_rate, _downsampling_rate);
    filter2.filter (*_pcl2_filtered);
    pclMapFilteredDownSampled.emplace( cloud2_name, _pcl2_filtered);

    cerr << "\n";
    cerr << FBLU("Downsampling Clouds... ") << "\n";
    int cloud1_size = pclMapFiltered[cloud1_name]->points.size();
    int cloud2_size = pclMapFiltered[cloud2_name]->points.size();
    cerr << FGRN("Fixed Cloud DownSampled: ") << cloud1_size << FGRN(" ==> ") <<
            pclMapFilteredDownSampled[cloud1_name]->points.size() << " DownSampling Factor: " << _downsampling_rate << "\n";
    cerr << FGRN("Moving Cloud DownSampled: ") << cloud2_size << FGRN(" ==> ") <<
            pclMapFilteredDownSampled[cloud2_name]->points.size() << " DownSampling Factor: " << _downsampling_rate << "\n" << "\n";
}

void PointCloudAlignerNew::finalRefinement(const string &cloud1_name, const string &cloud2_name){

    PCLPointCloudXYZ::Ptr _pcl1_xyz( new PCLPointCloudXYZ() );
    for( PCLptXYZRGB pt : pclMapFilteredDownSampled[cloud1_name]->points ){
        PCLptXYZ pt_xyz;
        pt_xyz.getVector3fMap() = pt.getVector3fMap();
        _pcl1_xyz->points.push_back( pt_xyz );
    }

    PCLKDtreeXYZ kdtree;
    kdtree.setInputCloud(_pcl1_xyz);
    vector<Vector3> fixed_pts, moving_pts;
    int iter = 0;
    while(iter < _max_iter_num){

        int id = 0;
        for( size_t i = 0; i < pclMapFilteredDownSampled[cloud2_name]->points.size(); ++i) {

                Vector3 query = pclMapFilteredDownSampled[cloud2_name]->points[i].getVector3fMap();
                pcl::PointXYZ searchPoint(query(0), query(1), query(2));
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if ( kdtree.radiusSearch (searchPoint, _search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                    Vector3 fix_pt = _pcl1_xyz->points[pointIdxRadiusSearch[0]].getVector3fMap();
                    fixed_pts.push_back( fix_pt );
                    moving_pts.push_back( query );
                }
            id++;
        }

        int refLen = fixed_pts.size();
        cpd::Matrix fixedRefImg(refLen,3);
        cpd::Matrix movingRefImg(refLen,3);

        for(unsigned int iter = 0; iter < refLen; ++iter){
            fixedRefImg(iter,0) = fixed_pts[iter](0); fixedRefImg(iter,1) = fixed_pts[iter](1); fixedRefImg(iter,2) = fixed_pts[iter](2);
            movingRefImg(iter,0) = moving_pts[iter](0); movingRefImg(iter,1) = moving_pts[iter](1); movingRefImg(iter,2) = moving_pts[iter](2);
        }

        cerr << FGRN("Iteration N° ") << iter << "\n";
        cpd::AffineResult refresult = cpd::affine(fixedRefImg, movingRefImg);

        Eigen::Matrix3f R = refresult.transform.cast<float> ();
        Eigen::Vector3f t = refresult.translation.cast<float> ();
        Transform FinalRefinementTf = Transform::Identity();
        FinalRefinementTf.translation() = t;
        FinalRefinementTf.linear() = R;

        pcl::transformPointCloud(*pclMap[cloud2_name], *pclMap[cloud2_name], FinalRefinementTf);
        pcl::transformPointCloud(*pclMapFiltered[cloud2_name], *pclMapFiltered[cloud2_name], FinalRefinementTf);

        if(_verbosity){
            cerr << "\n" << FYEL("Initial Alignment  Affine Matrix") << "\n";
            cerr << refresult.transform << "\n";
            cerr << FYEL("Initial Alignment Translation Vector") << "\n";
            cerr << refresult.translation.transpose() << "\n" << "\n";
        }


        _R = R*_R;
        _t = R*_t + t;

        fixed_pts.clear();
        moving_pts.clear();
        iter++;
    }
}

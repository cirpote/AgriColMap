#include "pointcloud_aligner_new.h"

using namespace std;

PointCloudAlignerNew::PointCloudAlignerNew() :_R(Matrix3::Identity()), _t(Vector3::Zero()) {}

void PointCloudAlignerNew::computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key,
                                                            const std::string& moving_cloud_key){

        if( getVerbosityLevel() )
            cerr << FBLU("Compute And Applying The Relative Initial Guess!") << "\n";

        getPointCloud(fixed_cloud_key)->planeNormalization();
        getPointCloud(moving_cloud_key)->planeNormalization();

        /*Vector3d mov_t, fix_t; Vector3 mov_q, fix_q;
        fix_t = getPointCloud(fixed_cloud_key)->getInitGuessT();
        fix_q = getPointCloud(fixed_cloud_key)->getInitGuessQ();
        mov_t = getPointCloud(moving_cloud_key)->getInitGuessT();
        mov_q = getPointCloud(moving_cloud_key)->getInitGuessQ();*/

        // Normalizing along the Yaw Axis the Fixed Cloud
        /*Transform Rot_z_fixed = Transform::Identity();
        Rot_z_fixed.rotate( Eigen::AngleAxisf( getPointCloud(fixed_cloud_key)->getInitGuessQ()(2)*(3.14/180), Vector3::UnitZ() ) );
        getPointCloud(fixed_cloud_key)->transformPointCloud(Rot_z_fixed);*/

        /*// Normalizing along the Roll and Pitch Axes the Moving Cloud
        Transform Rot_xy_moving = Transform::Identity();
        Rot_xy_moving.rotate( Eigen::AngleAxisf( getPointCloud(moving_cloud_key)->getInitGuessQ()(0)*(3.14/180), Vector3::UnitX() ) );
        Rot_xy_moving.rotate( Eigen::AngleAxisf( getPointCloud(moving_cloud_key)->getInitGuessQ()(1)*(3.14/180), Vector3::UnitY() ) );
        getPointCloud(moving_cloud_key)->transformPointCloud(Rot_xy_moving);

        // Normalizing along the Yaw Axes the Moving Cloud
        Transform Rot_z_moving = Transform::Identity();
        Rot_z_moving.rotate( Eigen::AngleAxisf( getPointCloud(moving_cloud_key)->getInitGuessQ()(2)*(3.14/180), Vector3::UnitZ() ) );
        getPointCloud(moving_cloud_key)->transformPointCloud(Rot_z_moving);*/

        // Normalized along X and Y axis the Fixed Cloud
        Vector3d diff_t( getPointCloud(moving_cloud_key)->getInitGuessT() - getPointCloud(fixed_cloud_key)->getInitGuessT() );
        _InitTfMap.emplace(moving_cloud_key, boost::shared_ptr<Transform>(new Transform(Transform::Identity())) );
        _InitTfMap[moving_cloud_key]->translation() << diff_t.cast<float>();
        //TinitGuess.translation() << diff_t.cast<float>();
        //getPointCloud(fixed_cloud_key)->transformPointCloud(TinitGuess);
        //setTinitGuess(TinitGuess);

        _init_mov_scale(0) *= _pclMap[moving_cloud_key]->getScaleNoise()(0);
        _init_mov_scale(1) *= _pclMap[moving_cloud_key]->getScaleNoise()(1);
        cerr << FBLU("InitMovScale Set to: ") << _init_mov_scale.transpose() << "\n";
        scalePointCloud( _init_mov_scale, "moving_cloud");

        if( getVerbosityLevel() ){
            cerr << FYEL("Fixed_Cloud Rot_z amount: ") << getPointCloud(fixed_cloud_key)->getInitGuessQ()(2) << "\n";
            cerr << FYEL("Moving_Cloud Rot_z amount: ") << getPointCloud(moving_cloud_key)->getInitGuessQ()(2) << "\n";
            cerr << FYEL("Init alignment guess: ") << _InitTfMap[moving_cloud_key]->translation().transpose() << "\n" << "\n";
        }
}       

void PointCloudAlignerNew::computeDensifiedPCLs( const std::string& fixed_cloud, const std::string& moving_cloud, const cv::Size& outp_img_size ){

    getPointCloud(fixed_cloud)->computeDesifiedPCL(outp_img_size, _InitTfMap[moving_cloud]->translation().head(2), getPackagePath(), Vector3i(255,0,0));

    float x_mean = 0.f; float y_mean = 0.f; int cloud_size = getPointCloud(moving_cloud)->getSize();
    for(unsigned int iter = 0; iter < cloud_size; ++iter ){
			x_mean += getPointCloud(moving_cloud)->getPointCloudAt(iter).x;
			y_mean += getPointCloud(moving_cloud)->getPointCloudAt(iter).y;
	}
    x_mean /= cloud_size;
    y_mean /= cloud_size;

    getPointCloud(moving_cloud)->computeDesifiedPCL(outp_img_size, Vector2(x_mean, y_mean), getPackagePath(), Vector3i(0,0,255));
}

void PointCloudAlignerNew::writeAffineTransformCPD(const string &iter, const string &cloud){
    Vector2 scale = getInitMovScale();

    std::ostringstream float_conv_x, float_conv_y;
    float_conv_x << scale(0);
    std::string sx(float_conv_x.str());
    std::replace( sx.begin(), sx.end(), '.', '_');

    float_conv_y << scale(1);
    std::string sy(float_conv_y.str());
    std::replace( sy.begin(), sy.end(), '.', '_');

    ofstream outputAffineTf;
    outputAffineTf.open (getPackagePath() + "/params/output/" + getMovingCloudPath() + "/" + "CPD_Comparison/" + getMovingCloudPath() + "_AffineGroundTruth_" + sx + "_" + sy + "_" + iter + ".txt");
    outputAffineTf << _R(0,0) << " " << _R(0,1) << " " << _R(0,2) << " " << _t(0) << " "
                   << _R(1,0) << " " << _R(1,1) << " " << _R(1,2) << " " << _t(1) << " "
                   << _R(2,0) << " " << _R(2,1) << " " << _R(2,2) << " " << _t(2) << " "
                   << scale(0) << " " << scale(1);
    outputAffineTf.close();
    cerr << FBLU("Ground Truth Affine Transform Written in: ") << getMovingCloudPath() + "/" + "CPD_Comparison/" + getMovingCloudPath() + "/params/output/" + getMovingCloudPath() + "_AffineGroundTruth_" + sx + "_" + sy + "_" + iter + ".txt" << "\n";
}

void PointCloudAlignerNew::writeAffineTransform(const string& iter, const string& cloud){

    Vector2 scale = getInitMovScale();
    Vector2 scaleNoise = _pclMap[cloud]->getScaleNoise();
    Vector2d translNoise = _pclMap[cloud]->getTranslNoise();
    float yawNoise = _pclMap[cloud]->getYawNoise();
    ofstream outputAffineTf;
    outputAffineTf.open (getPackagePath() + "/params/output/results/" + getMovingCloudPath() +
                         "_AffineGroundTruth_" + iter + "_" + to_string( (scaleNoise-Vector2(1,1)).norm() ) + "_" +
                         to_string( translNoise.norm() ) + "_" + to_string( yawNoise ) + ".txt");
    outputAffineTf << _R(0,0) << " " << _R(0,1) << " " << _R(0,2) << " " << _t(0) << " "
                   << _R(1,0) << " " << _R(1,1) << " " << _R(1,2) << " " << _t(1) << " "
                   << _R(2,0) << " " << _R(2,1) << " " << _R(2,2) << " " << _t(2) << " "
                   << scale(0) << " " << scale(1) << " " << scaleNoise(0) << " "
                   << scaleNoise(1) << " " << translNoise(0) << " "
                   << translNoise(1) << " " << yawNoise;
    outputAffineTf.close();
    cerr << FBLU("Ground Truth Affine Transform Written in: ") << getPackagePath() + "/params/output/" +
                 getMovingCloudPath() + "/" + getMovingCloudPath() + "_AffineGroundTruth_" +
                 iter + "_" + to_string( scaleNoise.norm() ) + "_" + to_string( translNoise.norm() ) +
                 "_" + to_string( yawNoise) + ".txt" << "\n";
}

void PointCloudAlignerNew::MatchGoICP(const string &cloud1_name, const string &cloud2_name){

    Vector2 _scaleNoise   = getPointCloud(cloud2_name)->getScaleNoise();
    Vector2d _TranslNoise = getPointCloud(cloud2_name)->getTranslNoise();
    float _YawNoise       = getPointCloud(cloud2_name)->getYawNoise();

    Vector3 transl(_TranslNoise(0),_TranslNoise(1),0.f);
    Matrix3 rot(Eigen::AngleAxisf(_YawNoise, Eigen::Vector3f::UnitZ()));
    getPointCloud(cloud2_name)->affineTransformPointCloud(rot, transl);
    getPointCloud(cloud2_name)->scalePointCloud(_scaleNoise);

    getPointCloud(cloud1_name)->downsamplePointCloud(.10);
    getPointCloud(cloud2_name)->downsamplePointCloud(.10);

    clock_t  clockBegin, clockEnd;
    POINT3D * pModel, * pData;
    GoICP goicp;

    int Nm = getPointCloud(cloud1_name)->getFilteredSize();
    pModel = (POINT3D *)malloc(sizeof(POINT3D) * Nm);

    for(unsigned int i = 0; i < Nm; i++)
    {
        pcl::PointXYZRGB pt = getPointCloud(cloud1_name)->getPointCloudFilteredAt(i);
        pModel[i].x = pt.x; pModel[i].y = pt.y; pModel[i].z = pt.z;
    }

    int Nd = getPointCloud(cloud2_name)->getFilteredSize();
    pData = (POINT3D *)malloc(sizeof(POINT3D) * Nd);
    for(unsigned int i = 0; i < Nd; i++)
    {
        pcl::PointXYZRGB pt = getPointCloud(cloud2_name)->getPointCloudFilteredAt(i);
        pData[i].x = pt.x; pData[i].y = pt.y; pData[i].z = pt.z;
    }

    goicp.MSEThresh = 0.005;
    goicp.initNodeRot.a = -3.1416;
    goicp.initNodeRot.b = -3.1416;
    goicp.initNodeRot.c = -3.1416;
    goicp.initNodeRot.w = 6.2832;
    goicp.initNodeTrans.x = -0.5;
    goicp.initNodeTrans.y = -0.5;
    goicp.initNodeTrans.z = -0.5;
    goicp.initNodeTrans.w = 1;
    goicp.trimFraction = 0.0;
    // If < 0.1% trimming specified, do no trimming
    if(goicp.trimFraction < 0.001)
    {
        goicp.doTrim = false;
    }
    goicp.dt.SIZE = 300;
    goicp.dt.expandFactor = 2.0;

    goicp.pModel = pModel;
    goicp.Nm = Nm;
    goicp.pData = pData;
    goicp.Nd = Nd;

    // Build Distance Transform
    cout << "Building Distance Transform..." << flush;
    clockBegin = clock();
    goicp.BuildDT();
    clockEnd = clock();
    cout << (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC << "s (CPU)" << endl;

    // Run GO-ICP
    std::cout << "Model ID: " << cloud1_name << " (" << goicp.Nm << "), Data ID: " << cloud2_name << " (" << goicp.Nd << ")" << "\n";
    std::cout << "Registering..." << "\n";
    clockBegin = clock();
    goicp.Register();
    clockEnd = clock();
    double time = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
    std::cout << "Optimal Rotation Matrix:" << "\n";
    std::cout << goicp.optR << "\n";
    std::cout << "Optimal Translation Vector:" << "\n";
    std::cout << goicp.optT << "\n";
    std::cout << "Finished in " << time << "\n";

    Matrix3 R( Matrix3::Identity() );
    Vector3 t( Vector3::Zero() );

    for (int32_t i=0; i<goicp.optR.m; i++) {
      for (int32_t j=0; j<goicp.optR.n; j++) {
         R(i,j) = goicp.optR.val[i][j];
      }
    }


    t(0) = goicp.optT.val[0][0];  t(1) = goicp.optT.val[1][0]; t(2) = goicp.optT.val[2][0];
    std::cout << R << "\n" << "\n";
    std::cout << t.transpose() << "\n";

    getPointCloud(cloud2_name)->affineTransformPointCloud(R, t);

}

void PointCloudAlignerNew::MatchCPD(const std::string &cloud1_name, const std::string &cloud2_name, const cv::Size &size, const Eigen::Vector2f &scale, const string &iter_num){

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_pcl( new pcl::PointCloud<pcl::PointXYZ> );
    for( unsigned int i = 0; i < getPointCloud(cloud1_name)->getFilteredSize(); ++i)
            xyz_pcl->points.push_back( pcl::PointXYZ(getPointCloud(cloud1_name)->getPointCloudFilteredAt(i).x,
                                                     getPointCloud(cloud1_name)->getPointCloudFilteredAt(i).y,
                                                     getPointCloud(cloud1_name)->getPointCloudFilteredAt(i).z) );
    kdtree.setInputCloud(xyz_pcl);

    Matrix3 R( Matrix3::Identity() );
    Vector3 t( Vector3::Zero() );
    //_R = _Rgt;
    //_t = _tgt;
    //_t += getPointCloud(cloud1_name)->randGpsInput;
    getPointCloud(cloud1_name)->downsamplePointCloud(.13);
    getPointCloud(cloud2_name)->downsamplePointCloud(.13);
    vector<Vector3> fixed_pts, moving_pts;

    float radius = 0.5; int MAX_ITER = 3;

    int iter = 0;
    while(iter < MAX_ITER){

        int id = 0;
        for( size_t i = 0; i < getPointCloud(cloud2_name)->getFilteredSize(); ++i) {

                Vector3 query = getPointCloud(cloud2_name)->getPointCloudFilteredAt(i).getVector3fMap();
                pcl::PointXYZ searchPoint(query(0), query(1), query(2));
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                    Vector3 fix_pt = xyz_pcl->points[pointIdxRadiusSearch[0]].getVector3fMap();
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

        R = refresult.transform.cast<float> ();
        t = refresult.translation.cast<float> ();
        getPointCloud(cloud2_name)->affineTransformPointCloud(R, t);


        _R = R*_R;
        _t = R*_t + t;

        fixed_pts.clear();
        moving_pts.clear();
        iter++;
    }

    if( getVerbosityLevel() ){
        cerr << "\n";
        std::cerr << FBLU("Final Affine Matrix: ") << "\n" << _R << "\n";
        std::cerr << FBLU("Final Translation: ") << _t.transpose() << "\n";
        std::cerr << FBLU("Initial Scale: ") << scale.transpose() << "\n";
    }

    writeAffineTransformCPD(iter_num, cloud2_name);
}

void PointCloudAlignerNew::WriteDenseOpticalFlow(const int& w, const int& h, const string& cloud, const string& iter){

    Vector2 scaleNoise = _pclMap[cloud]->getScaleNoise();
    Vector2d translNoise = _pclMap[cloud]->getTranslNoise();
    float yawNoise = _pclMap[cloud]->getYawNoise();
    FImage u, v;
    cpm.Match2Flow(matches, u, v, w, h);
    std::string img_path = getPackagePath() + "/params/output/dense_optical_flow/" + getMovingCloudPath() + iter + "_" + to_string( (scaleNoise-Vector2(1,1)).norm() ) + "_" +
            to_string( translNoise.norm() ) + "_" + to_string( yawNoise ) + ".png";
    OpticFlowIO::SaveFlowAsImage(img_path.c_str(), u.pData, v.pData, w, h);
}

void PointCloudAlignerNew::showDOFCorrespondeces(const int& len, const std::string& cloud1_name, const std::string& cloud2_name, const cv::Size& size){

    cv::Size draw_img_size( size.width*2, size.height );
    cv::Mat drawImg( draw_img_size, CV_8UC3, cv::Scalar(0,0,0) );
    getPointCloud(cloud1_name)->getRGBImg().copyTo( drawImg( cv::Rect(0, 0, size.width, size.height) )  );
    getPointCloud(cloud2_name)->getRGBImg().copyTo( drawImg( cv::Rect(size.width, 0, size.width, size.height) )  );

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

void PointCloudAlignerNew::computeAndApplyDOFTransform(const std::string& cloud1_name, const std::string& cloud2_name, int& len){

    cv::Mat fixedXYZImg = getPointCloud(cloud1_name)->getXYZImg();
    cv::Mat movingXYZImg = getPointCloud(cloud2_name)->getXYZImg();

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
    getPointCloud(cloud2_name)->affineTransformPointCloud(R, t);
}

void PointCloudAlignerNew::downsamplePointClouds(const std::string& cloud1_name, const std::string& cloud2_name){

    cerr << "\n";
    cerr << FBLU("Downsampling Clouds... ") << "\n";
    int cloud1_size = getPointCloud(cloud1_name)->getFilteredSize();
    int cloud2_size = getPointCloud(cloud2_name)->getFilteredSize();
    getPointCloud(cloud1_name)->downsamplePointCloud(_downsampling_rate);
    getPointCloud(cloud2_name)->downsamplePointCloud(_downsampling_rate);
    cerr << FGRN("Fixed Cloud DownSampled: ") << cloud1_size << FGRN(" ==> ") <<
            getPointCloud(cloud1_name)->getFilteredSize() << " DownSampling Factor: " << _downsampling_rate << "\n";
    cerr << FGRN("Moving Cloud DownSampled: ") << cloud2_size << FGRN(" ==> ") <<
            getPointCloud(cloud2_name)->getFilteredSize() << " DownSampling Factor: " << _downsampling_rate << "\n" << "\n";
}

void PointCloudAlignerNew::finalRefinement(const string &cloud1_name, const string &cloud2_name, const Eigen::Vector2f& scale){

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_pcl( new pcl::PointCloud<pcl::PointXYZ> );
    for( unsigned int i = 0; i < getPointCloud(cloud1_name)->getFilteredSize(); ++i)
            xyz_pcl->points.push_back( pcl::PointXYZ(getPointCloud(cloud1_name)->getPointCloudFilteredAt(i).x,
                                                     getPointCloud(cloud1_name)->getPointCloudFilteredAt(i).y,
                                                     getPointCloud(cloud1_name)->getPointCloudFilteredAt(i).z) );
    kdtree.setInputCloud(xyz_pcl);
    vector<Vector3> fixed_pts, moving_pts;
    int iter = 0;
    while(iter < _max_iter_num){

        int id = 0;
        for( size_t i = 0; i < getPointCloud(cloud2_name)->getFilteredSize(); ++i) {

                Vector3 query = getPointCloud(cloud2_name)->getPointCloudFilteredAt(i).getVector3fMap();
                pcl::PointXYZ searchPoint(query(0), query(1), query(2));
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if ( kdtree.radiusSearch (searchPoint, _search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                    Vector3 fix_pt = xyz_pcl->points[pointIdxRadiusSearch[0]].getVector3fMap();
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
        getPointCloud(cloud2_name)->affineTransformPointCloud(R, t);

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

FImage PointCloudAlignerNew::computeSiftCorrespondeces(const std::string& cloud1_name, const std::string& cloud2_name, const MatchingType &type){

    cv::Mat cloud1Rgb = getPointCloud(cloud1_name)->getExGImg();
    cv::Mat cloud2Rgb = getPointCloud(cloud2_name)->getExGImg();
    std::vector<cv::KeyPoint> keypointsCloud1, keypointsCloud2;
    cv::Mat descriptorsCloud1, descriptorsCloud2;

    cv::BFMatcher bf_matcher(cv::NORM_L2, false);
    cv::FlannBasedMatcher flann_matcher();
    std::vector< cv::DMatch > matches;
    std::vector< std::vector< cv::DMatch> > matches_knn;

    cv::Ptr<cv::Feature2D> f2dSURF = cv::xfeatures2d::SURF::create(5);
    cv::Ptr<cv::Feature2D> f2dSIFT = cv::xfeatures2d::SIFT::create();// 25000, 3, 0.03, 5, 1.6 );
    cv::Ptr<cv::FeatureDetector> f2dORB = cv::ORB::create(10000);
    cv::Ptr<cv::FastFeatureDetector> FASTdetector = cv::FastFeatureDetector::create(5, true, cv::FastFeatureDetector::TYPE_9_16);
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> BRIEFextractor = cv::xfeatures2d::BriefDescriptorExtractor::create(64);

    switch (type) {
        case MatchingType::SURF:
            f2dSURF->detect( cloud1Rgb, keypointsCloud1 );
            f2dSURF->detect( cloud2Rgb, keypointsCloud2 );
            f2dSURF->compute( cloud1Rgb, keypointsCloud1, descriptorsCloud1 );
            f2dSURF->compute( cloud2Rgb, keypointsCloud2, descriptorsCloud2 );
            bf_matcher.knnMatch(descriptorsCloud1, descriptorsCloud2, matches_knn, 4);
            for( std::vector< cv::DMatch > matches_vec : matches_knn){
                for( cv::DMatch match : matches_vec )
                    matches.push_back(match);
            }
            break;
        case MatchingType::SIFT:
            f2dSIFT->detect( cloud1Rgb, keypointsCloud1 );
            f2dSIFT->detect( cloud2Rgb, keypointsCloud2 );
            f2dSIFT->compute( cloud1Rgb, keypointsCloud1, descriptorsCloud1 );
            f2dSIFT->compute( cloud2Rgb, keypointsCloud2, descriptorsCloud2 );
            bf_matcher.knnMatch(descriptorsCloud1, descriptorsCloud2, matches_knn, 4);
            for( std::vector< cv::DMatch > matches_vec : matches_knn){
                for( cv::DMatch match : matches_vec )
                    matches.push_back(match);
            }
            break;
        case MatchingType::ORB:
            bf_matcher = cv::BFMatcher(cv::NORM_HAMMING2, false);
            f2dORB->detect( cloud1Rgb, keypointsCloud1 );
            f2dORB->detect( cloud2Rgb, keypointsCloud2 );
            f2dORB->compute( cloud1Rgb, keypointsCloud1, descriptorsCloud1 );
            f2dORB->compute( cloud2Rgb, keypointsCloud2, descriptorsCloud2 );
            bf_matcher.match( descriptorsCloud1, descriptorsCloud2, matches );
            break;
        case MatchingType::FAST_BRIEF:
            bf_matcher = cv::BFMatcher(cv::NORM_HAMMING2, false);
            FASTdetector->detect( cloud1Rgb, keypointsCloud1 );
            FASTdetector->detect( cloud2Rgb, keypointsCloud2 );
            BRIEFextractor->compute( cloud1Rgb, keypointsCloud1, descriptorsCloud1 );
            BRIEFextractor->compute( cloud2Rgb, keypointsCloud2, descriptorsCloud2 );
            bf_matcher.match( descriptorsCloud1, descriptorsCloud2, matches );
        default:
            break;
    }

    cv::Mat matchesImg;
    cv::drawMatches(cloud1Rgb,keypointsCloud1,cloud2Rgb,keypointsCloud2,matches,matchesImg);
    cv::imshow("Sift Matches", matchesImg);
    cv::waitKey(0);
    cv::destroyAllWindows();

    FImage outmatches(4, matches.size());

    for (int i = 0; i < matches.size(); i++){
            outmatches[4 * i + 0] = keypointsCloud1.at( matches.at(i).queryIdx ).pt.x;
            outmatches[4 * i + 1] = keypointsCloud1.at( matches.at(i).queryIdx ).pt.y;
            outmatches[4 * i + 2] = keypointsCloud2.at( matches.at(i).trainIdx ).pt.x;
            outmatches[4 * i + 3] = keypointsCloud2.at( matches.at(i).trainIdx ).pt.y;
    }

    return outmatches;

}

void PointCloudAlignerNew::Match( const std::string& cloud1_name, const std::string& cloud2_name,
                               const Eigen::Vector2f& scale, const string& iter_num, const cv::Size& size ){
    
    cpm.SetParams(_dense_optical_flow_step, _useVisualFeatures, _useGeometricFeatures);
    img1.imcopy( getPointCloud(cloud1_name)->getExGImg() );
    img2.imcopy( getPointCloud(cloud2_name)->getExGImg() );
    img1Cloud.imcopy( getPointCloud(cloud1_name)->getXYZImg() );
    img2Cloud.imcopy( getPointCloud(cloud2_name)->getXYZImg() );

    //FImage matches = computeSiftCorrespondeces(cloud1_name, cloud2_name, MatchingType::FAST_BRIEF);
    cpm.Matching(img1, img1Cloud, img2, img2Cloud, matches);

    if( _storeDenseOptFlw )
        WriteDenseOpticalFlow(img1.width(), img1.height(), cloud2_name, iter_num);

    cpm.VotingScheme(matches, filteredMatches, getPointCloud(cloud1_name)->getRGBImg(), getPointCloud(cloud2_name)->getRGBImg());

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
    finalRefinement(cloud1_name, cloud2_name, scale);
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

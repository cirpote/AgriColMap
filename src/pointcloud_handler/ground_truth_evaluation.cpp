#include "ground_truth_evaluation.h"

using namespace std;

GroundTruthEvaluation::GroundTruthEvaluation() : viewer( new pcl::visualization::PCLVisualizer ("PointCloud Viewer") ) {

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(0.f, 0.f, 100.f, 0.f, 1.f, 0.f);
    _T = Eigen::Vector4f(1.f,1.f,1.f,1.f).asDiagonal();
}

void GroundTruthEvaluation::computeAndApplyInitialRelativeGuess(const std::string& fixed_cloud_key,
                                                                const std::string& moving_cloud_key){

    if( getVerbosityLevel() )
        cerr << FBLU("Compute And Applying The Relative Initial Guess!") << "\n";

    Vector3 mean_moving(0.f, 0.f, 0.f);
    Vector3 mean_fixed(0.f, 0.f, 0.f);
    for(unsigned int i = 0; i < getPointCloud(moving_cloud_key)->getSize(); ++i)
        mean_moving += getPointCloud(moving_cloud_key)->getPointCloudAt(i).getVector3fMap();

    for(unsigned int i = 0; i < getPointCloud(fixed_cloud_key)->getSize(); ++i)
        mean_fixed += getPointCloud(fixed_cloud_key)->getPointCloudAt(i).getVector3fMap();


    mean_moving /= getPointCloud(moving_cloud_key)->getSize();
    mean_fixed /= getPointCloud(fixed_cloud_key)->getSize();

    Transform norm_T_mov( Transform::Identity() );
    norm_T_mov.translation() << -mean_moving;
    getPointCloud(moving_cloud_key)->transformPointCloud(norm_T_mov);

    Transform norm_T_fix( Transform::Identity() );
    norm_T_fix.translation() << -mean_fixed;
    getPointCloud(fixed_cloud_key)->transformPointCloud(norm_T_fix);

    Vector3 fix_q, fix_t, mov_q, mov_t;
    fix_t = getPointCloud(fixed_cloud_key)->getInitGuessT();
    fix_q = getPointCloud(fixed_cloud_key)->getInitGuessQ();
    mov_t = getPointCloud(moving_cloud_key)->getInitGuessT();
    mov_q = getPointCloud(moving_cloud_key)->getInitGuessQ();

    // Normalizing along the Yaw Axis the Fixed Cloud
    Transform Rot_z_fixed = Transform::Identity();
    Rot_z_fixed.rotate( Eigen::AngleAxisf( fix_q(2)*(3.14/180), Vector3::UnitZ() ) );
    getPointCloud(fixed_cloud_key)->transformPointCloud(Rot_z_fixed);

    // Normalizing along the Roll and Pitch Axes the Moving Cloud
    Transform Rot_xy_moving = Transform::Identity();
    Rot_xy_moving.rotate( Eigen::AngleAxisf( mov_q(0)*(3.14/180), Vector3::UnitX() ) );
    Rot_xy_moving.rotate( Eigen::AngleAxisf( mov_q(1)*(3.14/180), Vector3::UnitY() ) );
    getPointCloud(moving_cloud_key)->transformPointCloud(Rot_xy_moving);

    // Normalizing along the Yaw Axes the Moving Cloud
    Transform Rot_z_moving = Transform::Identity();
    Rot_z_moving.rotate( Eigen::AngleAxisf( mov_q(2)*(3.14/180), Vector3::UnitZ() ) );
    getPointCloud(moving_cloud_key)->transformPointCloud(Rot_z_moving);

    Transform TinitGuess = Transform::Identity();
    TinitGuess.translation() = Vector3( getPointCloud(moving_cloud_key)->getInitGuessT() - getPointCloud(fixed_cloud_key)->getInitGuessT() );
    setTinitGuess(TinitGuess);

    if( getVerbosityLevel() ){
        cerr << FYEL("Fixed_Cloud Rot_z amount: ") << fix_q(2) << "\n";
        cerr << FYEL("Moving_Cloud Rot_z amount: ") << mov_q(2) << "\n";
        cerr << FYEL("Init alignment guess: ") << TinitGuess.translation().transpose() << "\n" << "\n";
    }

}       

void GroundTruthEvaluation::showCloud( const string& cloud_to_show ){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

    if( getPointCloud(cloud_to_show)->getFilteredSize() ){
        for(unsigned int i = 0; i < getPointCloud(cloud_to_show)->getFilteredSize(); ++i){
            cloud->points.push_back( getPointCloud(cloud_to_show)->getPointCloudFilteredAt(i) );
        }
    } else if( getPointCloud(cloud_to_show)->getSize() ) {
        for(unsigned int i = 0; i < getPointCloud(cloud_to_show)->getSize(); ++i){
            cloud->points.push_back( getPointCloud(cloud_to_show)->getPointCloudAt(i) );
        }
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> RGB(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, RGB, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_to_show);

}

void GroundTruthEvaluation::spingUntilDeath(){

    viewer->setSize(1920, 1080);
    while(!viewer->wasStopped())
        viewer->spin();
}

void GroundTruthEvaluation::computeDensifiedPCLs( const std::string& fixed_cloud, const std::string& moving_cloud, const cv::Size& outp_img_size ){

    getPointCloud(fixed_cloud)->computeDesifiedPCL(outp_img_size, getTinitGuess().translation().head(2), getPackagePath(), Vector3i(255,0,0));

    float x_mean = 0.f; float y_mean = 0.f; int cloud_size = getPointCloud(moving_cloud)->getSize();
    for(unsigned int iter = 0; iter < cloud_size; ++iter ){
        x_mean += getPointCloud(moving_cloud)->getPointCloudAt(iter).x;
        y_mean += getPointCloud(moving_cloud)->getPointCloudAt(iter).y;
    }
    x_mean /= cloud_size;
    y_mean /= cloud_size;

    getPointCloud(moving_cloud)->computeDesifiedPCL(outp_img_size, Vector2(x_mean, y_mean), getPackagePath(), Vector3i(0,255,0));
}

void GroundTruthEvaluation::setAffineTransform(const Eigen::Matrix3f &R, const Eigen::Vector3f &t){

    _T.block<3,3>(0,0) = R;
    _T.block<3,1>(0,3) = t;
}

void GroundTruthEvaluation::writeAffineTransform(const Eigen::Vector2f& scale){

    ofstream outputAffineTf;
    outputAffineTf.open (getPackagePath() + "/params/output/" + getMovingCloudPath() + "_AffineGroundTruth.txt");
    outputAffineTf << _T(0,0) << " " << _T(0,1) << " " << _T(0,2) << " " << _T(0,3) << " "
                   << _T(1,0) << " " << _T(1,1) << " " << _T(1,2) << " " << _T(1,3) << " "
                   << _T(2,0) << " " << _T(2,1) << " " << _T(2,2) << " " << _T(2,3) << " "
                   << scale(0) << " " << scale(1);
    outputAffineTf.close();
    cerr << FBLU("Ground Truth Affine Transform Written in: ") << getPackagePath() + "/params/output/" + getMovingCloudPath() + "_AffineGroundTruth.txt" << "\n";
}

void GroundTruthEvaluation::Match( const std::string& cloud1_name, const std::string& cloud2_name, const cv::Size& size, const Eigen::Vector2f& scale ){
    
    cv::Size draw_img_size( size.width*2, size.height );
    cv::Mat drawImg( draw_img_size, CV_8UC3, cv::Scalar(0,0,0) );
    getPointCloud(cloud1_name)->getRGBImg().copyTo( drawImg( cv::Rect(0, 0, size.width, size.height) )  );
    getPointCloud(cloud2_name)->getRGBImg().copyTo( drawImg( cv::Rect(size.width, 0, size.width, size.height) )  );

    cv::Point2i pt(0,0);
    cv::namedWindow("Ground Truth Window");
    cv::setMouseCallback("Ground Truth Window", onMouse, (void*)&pt);

    vector<cv::Point2i> fix_pts, mov_pts;

    int counter = 1;
    while(true)
    {
        cv::imshow("Ground Truth Window", drawImg);
        cv::waitKey(0);

        if(pt.x < 0 && pt.y < 0)
            break;

        if( counter%2 ) {
            cv::circle(drawImg, pt, 6, cv::Scalar(255,0,0), 3);
            fix_pts.push_back(pt);
        } else if ( !(counter%2) ) {
            cv::circle(drawImg, pt, 6, cv::Scalar(0,255,0), 3);
            mov_pts.push_back(pt);
        }
        counter++;
    }

    cv::destroyWindow("Ground Truth Window");
    cv::Mat fixedXYZImg = getPointCloud(cloud1_name)->getXYZImg();
    cv::Mat movingXYZImg = getPointCloud(cloud2_name)->getXYZImg();
    int len = fix_pts.size();
    cpd::Matrix fixMat(len,3);
    cpd::Matrix movMat(len,3);
    for(unsigned int iter = 0; iter < len; ++iter){
        fixMat(iter,0) = fixedXYZImg.at<cv::Vec3f>(fix_pts[iter])[0];
        fixMat(iter,1) = fixedXYZImg.at<cv::Vec3f>(fix_pts[iter])[1];
        fixMat(iter,2) = fixedXYZImg.at<cv::Vec3f>(fix_pts[iter])[2];
        movMat(iter,0) = movingXYZImg.at<cv::Vec3f>(mov_pts[iter])[0];
        movMat(iter,1) = movingXYZImg.at<cv::Vec3f>(mov_pts[iter])[1];
        movMat(iter,2) = movingXYZImg.at<cv::Vec3f>(mov_pts[iter])[2];
    }

    cerr << "\n";
    cerr << FBLU("Computing Initial Alignment From Dense Optical Flow Correspondeces...") << "\n";
    cpd::AffineResult result = cpd::affine(fixMat, movMat);
    cerr << FGRN("Initial Alignment  Affine Matrix") << "\n";
    cerr << result.transform << "\n";
    cerr << FGRN("Initial Alignment Translation Vector") << "\n";
    cerr << result.translation.transpose() << "\n" << "\n";

    Eigen::Matrix3f R = result.transform.cast<float> ();
    Eigen::Vector3f t = result.translation.cast<float> ();

    getPointCloud(cloud2_name)->affineTransformPointCloud(R, t);

    setAffineTransform(R, t);
    writeAffineTransform(scale);
}

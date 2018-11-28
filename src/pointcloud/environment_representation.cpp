#include "environment_representation.h"

EnvironmentRepresentation::EnvironmentRepresentation(const std::string& cloudName) : _cloudName(cloudName) {} //, _scaleNoise(1.f, 1.f) {}

void EnvironmentRepresentation::loadFromPCLcloud( const PCLPointCloudXYZRGB::Ptr& pointCloud, const float& square_size ){

    _square_size = square_size;
    pcl::getMinMax3D(*pointCloud, minPt, maxPt);

    _width = std::ceil( (maxPt.x - minPt.x) / square_size );
    _height = std::ceil( (maxPt.y - minPt.y) / square_size );
    x_coord = minPt.x;
    y_coord = maxPt.y;
    altitude_scale = 255.0 / (maxPt.z - minPt.z);

    PCLptXYZRGB init_pt;
    init_pt.x = 0.f; init_pt.y = 0.f; init_pt.z = 0.f;

    std::vector<PCLptXYZRGB> init_pt_vector(50, init_pt);
    _gridMap = std::vector< std::vector<PCLptXYZRGB> >( _width * _height, init_pt_vector );

    PCLPointCloudXYZ::Ptr organized_gridmap_pcl( new PCLPointCloudXYZ(1, _width * _height) );

    for(unsigned int r = 0; r < _height; ++r){
        for(unsigned int c = 0; c < _width; ++c){
            organized_gridmap_pcl->points[c + r * _width].x = x_coord + (float) c * square_size;
            organized_gridmap_pcl->points[c + r * _width].y = y_coord - (float) r * square_size;
            organized_gridmap_pcl->points[c + r * _width].z = 0.f;
        }
    }

    PCLKDtreeXYZ kdTreeXYZ;
    kdTreeXYZ.setInputCloud(organized_gridmap_pcl);
    float radius = 0.01;
    for( PCLptXYZRGB pt : pointCloud->points){
        PCLptXYZ searchPoint( pt.x, pt.y, 0 );
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdTreeXYZ.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
            float x_idx = organized_gridmap_pcl->points[pointIdxRadiusSearch[0]].x;
            float y_idx = organized_gridmap_pcl->points[pointIdxRadiusSearch[0]].y;
            float c_idx = ( (x_idx - x_coord) / square_size );
            float r_idx = ( (y_coord - y_idx) / square_size );
            _gridMap[c_idx + r_idx * _width].push_back(pt);
        }
    }

}

PCLptXYZRGB EnvironmentRepresentation::computeAveragePoint(std::vector<PCLptXYZRGB>& ptVec,
                                                           const unsigned int& col,
                                                           const unsigned int& row){

    PCLptXYZRGB pt;
    float sum = 0.f; float x = 0.f; float y = 0.f;
    float z = 0.f; float r = 0.f; float g = 0.f; float b = 0.f;
    Vector2 nom_pt(x_coord + col * _square_size, y_coord - row * _square_size);
    float radius = 0.01;
    for( PCLptXYZRGB pt : ptVec ){
        if(pt.getVector3fMap().norm() > 1e-4){
            float weight = ( nom_pt - pt.getVector3fMap().head(2) ).norm()/radius;
            sum += weight;
            x += pt.x*weight;
            y += pt.y*weight;
            z += pt.z*weight;
            r += (float)pt.r*weight;
            g += (float)pt.g*weight;
            b += (float)pt.b*weight;
        }
    }
    pt.x = x/sum;
    pt.y = y/sum;
    pt.z = z/sum;
    pt.r = (int)(r/sum);
    pt.g = (int)(g/sum);
    pt.b = (int)(b/sum);
    return pt;
}


void EnvironmentRepresentation::computeMMGridMap(){

    // Image size is flipped for the sake of better visualization of the results
    exgImg = cv::Mat( cv::Size(_height, _width), CV_8UC1, cv::Scalar(0) );
    elevImg = cv::Mat( cv::Size(_height, _width), CV_8UC1, cv::Scalar(0) );
    xyzImg = cv::Mat( cv::Size(_height, _width), CV_32FC3, cv::Scalar(0,0,0) );
    exgImgColor = cv::Mat( cv::Size(_height, _width), CV_8UC3, cv::Scalar(0,0,0) );
    int iter = 0;
    for(unsigned int r = 0; r < _height; ++r){
        for(unsigned int c = 0; c < _width; ++c, ++iter){
            PCLptXYZRGB pt = computeAveragePoint(_gridMap[iter], c, r);
            int ExG = computeExGforXYZRGBPoint(pt);
            exgImg.at<uchar>(c,r) = ExG;
            exgImgColor.at<cv::Vec3b>(c,r)[1] = ExG;
            elevImg.at<uchar>(c,r) = altitude_scale * ( pt.z - minPt.z );
            xyzImg.at<cv::Vec3f>(c,r)[0] = pt.x;
            xyzImg.at<cv::Vec3f>(c,r)[1] = pt.y;
            xyzImg.at<cv::Vec3f>(c,r)[2] = pt.z;
        }
    }

    cv::normalize(xyzImg, xyzImgUChar, 255, 0, cv::NORM_MINMAX);
    xyzImgUChar.convertTo(xyzImgUChar, CV_8UC3);

    cv::imshow("exgImgColor", exgImgColor);
    cv::imshow("elevImg", elevImg);
    cv::imshow("xyzImgUChar", xyzImgUChar);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

/*void PointCloud::copyFrom(const PCLptXYZRGB_Vector& pcl_data,
                          const Vector3d& t,
                          const Vector3& q){

    _PointCloud.reserve( pcl_data.size() );
    _PointCloud = pcl_data;

    _PointCloudData.reserve( pcl_data.size() );
    for(unsigned int i = 0; i < _PointCloudData.size(); ++i)
        _PointCloudData[i] = _PointData();

    _init_guess_t = t;
    _init_guess_q = q;
}

bool PointCloud::getPointCloudFilteredAtWithRange(const int &i, const float &range, PCLptXYZRGB &pt){

    Vector2 mean(0,0);
    for(unsigned int iter = 0; iter < _PointCloudFiltered.size(); ++iter){
        mean += Vector2( _PointCloudFiltered[iter].getArray3fMap().head(2) );
    }
    mean /= _PointCloudFiltered.size();

    if( (Vector2(_PointCloudFiltered[i].getArray3fMap().head(2)) - mean).norm() < range ){
        pt = _PointCloudFiltered[i];
        return true;
    } else {
        return false;
    }
}

void PointCloud::planeNormalization(){

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    PCLsegmentationXYZ seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    // Computing Moving PointCloud Plane Equation
    PCLPointCloudXYZ::Ptr temp_xyz_pcl( new PCLPointCloudXYZ() );
    for( unsigned int i = 0; i < _PointCloud.size(); i+=10){
        PCLptXYZ curr_pt;
        curr_pt.getArray3fMap() = _PointCloud[i].getArray3fMap();
        temp_xyz_pcl->points.push_back( curr_pt );
    }

    seg.setInputCloud (temp_xyz_pcl);
    seg.segment (*inliers, *coefficients);

    Vector3 ez;
    if( coefficients->values[2] > 0)
        ez = Vector3(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    else if( coefficients->values[2] < 0 )
        ez = Vector3(-coefficients->values[0], -coefficients->values[1], -coefficients->values[2]);

    Vector3 ez_B = ez / ez.norm();

    float yaw_des = _init_guess_q(2)*(3.14/180);
    Vector3 ey_C( -sin(yaw_des), cos(yaw_des), 0 );

    Vector3 ex_B = ey_C.cross( ez_B ) / ( ey_C.cross( ez_B ) ).norm();
    Vector3 ey_B = ez_B.cross( ex_B ) / ( ez_B.cross( ex_B ) ).norm();

    Matrix3 A; A.col(0) = ex_B; A.col(1) = ey_B; A.col(2) = ez_B;

    Transform R = Transform::Identity();
    R.rotate(A.transpose());
    R.translation() << Vector3(0,0,coefficients->values[3]);

    R.translation() << Vector3(0,0,coefficients->values[3]);
    transformPointCloud(R);

    return;
}

void PointCloud::downsamplePointCloud(const float &downsampl_range){

    // Generating temporary PCL PointCloud
    PCLPointCloudXYZRGB::Ptr temp_xyzrgb_pcl( new PCLPointCloudXYZRGB() );
    PCLPointCloudXYZRGB::Ptr filtered_xyzrgb_pcl( new PCLPointCloudXYZRGB() );
    for( unsigned int i = 0; i < _PointCloudFiltered.size(); ++i){
        PCLptXYZRGB curr_pt;
        curr_pt.getArray3fMap() = _PointCloudFiltered[i].getArray3fMap();
        curr_pt.r = _PointCloudFiltered[i].r;
        curr_pt.g = _PointCloudFiltered[i].g;
        curr_pt.b = _PointCloudFiltered[i].b;
        temp_xyzrgb_pcl->points.push_back( curr_pt );
    }

    // Filtering the PCL PointCloud
    PCLvoxelGridXYZRGB filter;
    filter.setInputCloud (temp_xyzrgb_pcl);
    filter.setLeafSize (downsampl_range, downsampl_range, downsampl_range);
    filter.filter (*filtered_xyzrgb_pcl);

    // Storing the values inside the _PointCloudFiltered Structure
    _PointCloudFiltered.clear();
    for(unsigned int iter = 0; iter < filtered_xyzrgb_pcl->size(); ++iter)
        _PointCloudFiltered.push_back( filtered_xyzrgb_pcl->points[iter] );

    // Generating temporary PCL PointCloud
    PCLPointCloudXYZRGB::Ptr temp_xyzrgb_pcl2( new PCLPointCloudXYZRGB() );
    PCLPointCloudXYZRGB::Ptr filtered_xyzrgb_pcl2( new PCLPointCloudXYZRGB() );
    for( unsigned int i = 0; i < _PointCloud.size(); ++i){
        PCLptXYZRGB curr_pt;
        curr_pt.getArray3fMap() = _PointCloud[i].getArray3fMap();
        curr_pt.r = _PointCloud[i].r;
        curr_pt.g = _PointCloud[i].g;
        curr_pt.b = _PointCloud[i].b;
        temp_xyzrgb_pcl2->points.push_back( curr_pt );
    }

    // Filtering the PCL PointCloud
    PCLvoxelGridXYZRGB filter2;
    filter2.setInputCloud (temp_xyzrgb_pcl2);
    filter2.setLeafSize (downsampl_range, downsampl_range, downsampl_range);
    filter2.filter (*filtered_xyzrgb_pcl2);

    // Storing the values inside the _PointCloudFiltered Structure
    _PointCloud.clear();
    for(unsigned int iter = 0; iter < filtered_xyzrgb_pcl2->size(); ++iter)
        _PointCloud.push_back( filtered_xyzrgb_pcl2->points[iter] );
}

void PointCloud::affineTransformPointCloud(const Eigen::Matrix3f &R, const Eigen::Vector3f &t){

    for(unsigned int i = 0; i < _PointCloud.size(); ++i)
        _PointCloud[i].getVector3fMap() = R*_PointCloud[i].getVector3fMap() + t;

    if( _PointCloudFiltered.size() ){
        for(unsigned int i = 0; i < _PointCloudFiltered.size(); ++i)
            _PointCloudFiltered[i].getVector3fMap() = R*_PointCloudFiltered[i].getVector3fMap() + t;
    }
}

void PointCloud::transformPointCloud(const Transform& tf){

    for(unsigned int i = 0; i < _PointCloud.size(); ++i)
        _PointCloud[i].getVector3fMap() = tf.rotation()*_PointCloud[i].getVector3fMap() + tf.translation();

    if( _PointCloudFiltered.size() ){
        for(unsigned int i = 0; i < _PointCloudFiltered.size(); ++i)
            _PointCloudFiltered[i].getVector3fMap() = tf.rotation()*_PointCloudFiltered[i].getVector3fMap() + tf.translation();
    }
}

void PointCloud::scalePointCloud(const Vector2& scale_factors){

    for(unsigned int i = 0; i < _PointCloud.size(); ++i){
        _PointCloud[i].x *= scale_factors(0);
        _PointCloud[i].y *= scale_factors(1);
    }

    if(_XYZImg.cols && _XYZImg.rows ){

        for(unsigned int c = 0; c < _ElevImg.cols; ++c){
            for(unsigned int r = 0; r < _ElevImg.rows; ++r){

                _XYZImg.at<cv::Vec3f>(r,c)[0] *= scale_factors(0);
                _XYZImg.at<cv::Vec3f>(r,c)[1] *= scale_factors(1);
            }
        }

    }

}

void PointCloud::computeDesifiedPCL(const cv::Size& outp_img_size, const Vector2& offset, const std::string& package_path, const Vector3i& color, const bool &take_higher){

    computePlanarKDTree();

    // Init Images
    _ExGImg = cv::Mat( outp_img_size, CV_8UC1, cv::Scalar(0));
    _ElevImg = cv::Mat( outp_img_size, CV_8UC1, cv::Scalar(0) );
    _XYZImg = cv::Mat( outp_img_size, CV_32FC3, cv::Scalar(0,0,0) );
    _RGBImg = cv::Mat( outp_img_size, CV_8UC3, cv::Scalar(0,0,0) );
    _ExGColorImg = cv::Mat ( outp_img_size, CV_8UC3, cv::Scalar(0,0,0) );

    computeImgs(offset, color, take_higher);

}

void PointCloud::computeFilteredPcl(const Vector3i &color){

    float r, b, g, sum, ExG = 0.f;
    for( PCLptXYZRGB pt : _PointCloud ){
        sum = (int)pt.r + (int)pt.g + (int)pt.b;
        r = (int)pt.r / sum;
        g = (int)pt.g / sum;
        b = (int)pt.b / sum;
        ExG = (2 * g - b - r) * 255.0;
        ExG = std::max(0.f, ExG);
        if(ExG > 50){
            PCLptXYZRGB new_pt;
            new_pt.getVector3fMap() = pt.getVector3fMap();
            new_pt.r = color(0); new_pt.g = color(1); new_pt.b = color(2);
            _PointCloudFiltered.push_back( new_pt );
        }
    }
}

PCLptXYZRGB PointCloud::computeAveragePoint(const std::vector<int>& Idx, const std::vector<float>& Radius, const float& range){

    PCLptXYZRGB pt;
    float sum = 0.f; float x = 0.f; float y = 0.f;
    float z = 0.f; float r = 0.f; float g = 0.f; float b = 0.f;
    for(unsigned int iter = 0; iter < Idx.size(); ++iter){
        float weight = Radius[iter]/range;
        sum += weight;
        x += _PointCloud[Idx[iter]].x*weight;
        y += _PointCloud[Idx[iter]].y*weight;
        z += _PointCloud[Idx[iter]].z*weight;
        r += (float)_PointCloud[Idx[iter]].r*weight;
        g += (float)_PointCloud[Idx[iter]].g*weight;
        b += (float)_PointCloud[Idx[iter]].b*weight;
    }
    pt.x = x/sum;
    pt.y = y/sum;
    pt.z = z/sum;
    pt.r = (int)(r/sum);
    pt.g = (int)(g/sum);
    pt.b = (int)(b/sum);
    return pt;
}

void PointCloud::computeImgs(const Vector2& offset, const Vector3i& color, const bool &take_higher){

    int size_mt = (20*_ElevImg.cols)/1000;
    float radius = 0.01; float min_z = 1000; float max_z = -1000;
    for(unsigned int c = 0; c < _ElevImg.cols; ++c){
        for(unsigned int r = 0; r < _ElevImg.rows; ++r){

            PCLptXYZ searchPoint( 0 + ( (float)r - _ElevImg.rows/2 ) * size_mt/_ElevImg.rows,
                                       0 + ( (float)c - _ElevImg.cols/2 ) * size_mt/_ElevImg.cols, 0.f );

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if ( planar_kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){

                PCLptXYZRGB pt;
                if(take_higher){
                    pt = getHighestPoint(pointIdxRadiusSearch, _PointCloud);
                } else {
                    pt = _PointCloud[pointIdxRadiusSearch[0]];
                }
                float z_ = pt.z;
                if( z_ < min_z )
                    min_z = z_;

                if( z_ > max_z )
                    max_z = z_;
            }

        }
    }

    cv::Mat _XYZImg_viz = _XYZImg.clone();
    int lut_scale = 255.0 / (max_z - min_z);
    for(unsigned int c = 0; c < _ExGImg.cols; ++c){
        for(unsigned int r = 0; r < _ExGImg.rows; ++r){

            PCLptXYZ searchPoint( offset(0) + ( (float)r - _ExGImg.rows/2 ) * size_mt/_ExGImg.rows,
                                       offset(1) + ( (float)c - _ExGImg.cols/2 ) * size_mt/_ExGImg.cols, 0.f );

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

             if ( planar_kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                PCLptXYZRGB pt;
                if(take_higher){
                     pt = getHighestPoint(pointIdxRadiusSearch, _PointCloud);
                } else {
                     pt = computeAveragePoint(pointIdxRadiusSearch, pointRadiusSquaredDistance, radius);
                     //pt = _PointCloud[pointIdxRadiusSearch[0]];
                }
                _ExGImg.at<uchar>(r,c) = computeExGforPoint( pt, _PointCloudFiltered, color );
                _ExGColorImg.at<cv::Vec3b>(r,c)[1] = _ExGImg.at<uchar>(r,c);

                // RGB Img
                _RGBImg.at<cv::Vec3b>(r,c)[0] = (int)pt.b;
                _RGBImg.at<cv::Vec3b>(r,c)[1] = (int)pt.g;
                _RGBImg.at<cv::Vec3b>(r,c)[2] = (int)pt.r;

                // XYZ Img
                _XYZImg.at<cv::Vec3f>(r,c)[0] = pt.x;
                _XYZImg.at<cv::Vec3f>(r,c)[1] = pt.y;
                _XYZImg.at<cv::Vec3f>(r,c)[2] = pt.z;

                _XYZImg_viz.at<cv::Vec3f>(r,c)[0] = pt.x/20;
                _XYZImg_viz.at<cv::Vec3f>(r,c)[1] = pt.y/20;
                if( _ExGImg.at<uchar>(r,c) > 30 ){
                    _XYZImg_viz.at<cv::Vec3f>(r,c)[2] = pt.z*10;
                } else {
                    _XYZImg_viz.at<cv::Vec3f>(r,c)[2] = pt.z;
                }

                // Elev Img
                float z_ = pt.z;
                float dist = lut_scale * (z_ - min_z);
                _ElevImg.at<uchar>(r,c) = (int)dist;
            }

        }
    }

    cv::normalize( _XYZImg_viz, _XYZImg_viz, 0, 255, cv::NORM_MINMAX, CV_32FC1 );
    _XYZImg_viz.convertTo(_XYZImg_viz, CV_8UC1);

    //cv::imshow("_ElevImg", _XYZImg_viz );
    //cv::imwrite("/home/ciro/Scrivania/height_map.png", _XYZImg_viz);
    //cv::imwrite("/home/ciro/Scrivania/rgb.png", _RGBImg);
    //cv::imshow("exgImg", _ExGColorImg);
    //cv::imwrite("/home/ciro/Scrivania/2.png", _ExGColorImg);
    //cv::waitKey(0);
    //cv::destroyAllWindows();
}

void PointCloud::computePlanarKDTree(){

    PCLPointCloudXYZ::Ptr xyz_pcl( new PCLPointCloudXYZ() );
    for( unsigned int i = 0; i < _PointCloud.size(); ++i)
            xyz_pcl->points.push_back( pcl::PointXYZ( _PointCloud[i].x, _PointCloud[i].y,  0.f) );

    // Neighbors within radius search
    planar_kdtree.setInputCloud (xyz_pcl);
}

void PointCloud::addNoise(const float& scaleMag, const float& TranslMag, const float& YawMag){

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
            FBLU(" _init_guess_t: ") << _init_guess_t.head(2).transpose() <<
            FBLU(" _init_guess_t + Noise: ")  << (_init_guess_t.head(2) + _TranslNoise).transpose() << "\n";
    _init_guess_t(0) += _TranslNoise(0);
    _init_guess_t(1) += _TranslNoise(1);

    // Translational Noise With Sampling over a Fixed Input Magnitude
    _YawNoise = YawMag + ( .025*YawMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * YawMag ) );
    cerr << FBLU("Yaw Rotational Noise: ") << _YawNoise << "\n";
    _init_guess_q(2) += _YawNoise;

    // Translational Noise With Sampling over a Fixed Input Magnitude
    Vector2 vS = Eigen::Rotation2Df( CircleSamplingAngle )*v;
    vS(0) *= scaleMag + (.025*scaleMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * scaleMag ));
    vS(1) *= scaleMag + (.025*scaleMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * scaleMag ));
    _scaleNoise += vS;

    cerr << FBLU(" Scale Noise: ") << vS.transpose() << FBLU(" Scale Norm: ") << vS.norm() << "\n";
}

void PointCloud::BrightnessEnhancement(const int& brightness, const bool& convertToBW){

    for( PCLptXYZRGB& pt : _PointCloud){
        pt.r = cv::saturate_cast<uchar>( pt.r + (uchar)brightness );
        pt.g = cv::saturate_cast<uchar>( pt.g + (uchar)brightness );
        pt.b = cv::saturate_cast<uchar>( pt.b + (uchar)brightness );

        if(convertToBW){
            float grayscale = .299*(float)( (int)pt.r ) + .587*(float)( (int)pt.g ) + .114*(float)( (int)pt.b );
            pt.r = (int)grayscale; pt.g = (int)grayscale; pt.b = (int)grayscale;
        }
    }
}*/

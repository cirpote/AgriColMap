#include "environment_representation.h"

EnvironmentRepresentation::EnvironmentRepresentation(const std::string& cloudName) : _cloudName(cloudName) {}

void EnvironmentRepresentation::loadFromPCLcloud(const std::shared_ptr<open3d::PointCloud> pointCloud,
                                                  const float& square_size,
                                                  const Vector2d &imgCenter,
                                                  const float& radius,
                                                  const cv::Size& gridMapSize){

    _square_size = square_size;
    minPt = pointCloud->GetMinBound();
    maxPt = pointCloud->GetMaxBound();

    _width = gridMapSize.width;
    _height = gridMapSize.height;

    minPt(0) = - ( _width / 2 ) * square_size + imgCenter(0);
    maxPt(1) = - ( _height / 2 ) * square_size + imgCenter(1);

    x_coord = minPt(0);
    y_coord = maxPt(1);
    altitude_scale = 255.0 / (maxPt(2) - minPt(2));

    _gridMap = std::vector< std::vector<PclPoint> >( _width * _height);
    KDTreeXYvector kdTreeXY_points(_width * _height);
    for(unsigned int r = 0; r < _height; ++r){
        for(unsigned int c = 0; c < _width; ++c){
            kdTreeXY_points[c + r * _width](0) = x_coord + (float) c * square_size;
            kdTreeXY_points[c + r * _width](1) = y_coord + (float) r * square_size;
        }
    }

    float leaf_range = 0.1;
    KDTreeXY* kd_tree = new KDTreeXY(kdTreeXY_points, leaf_range);

    unsigned int iter = 0;
    for( Vector3d pt : pointCloud->points_){  
      KDTreeXYpoint query_point = pt.head(2);
      KDTreeXYpoint answer;
      int index;

      float approx_distance = kd_tree->findNeighbor(answer, index, query_point, radius);
      if (approx_distance > 0) {
          float c_idx = ( (answer(0) - x_coord) / square_size );
          float r_idx = ( (answer(1) - y_coord) / square_size );
          int _gridMapIndex = c_idx + r_idx * _width;

          PclPoint curr_pt_;
          curr_pt_.xyz_ = pt;
          curr_pt_.color_ = pointCloud->colors_[iter];
          _gridMap[_gridMapIndex].push_back(curr_pt_);
      }
      iter++;
    }
}

PclPoint EnvironmentRepresentation::computeAveragePoint(std::vector<PclPoint>& ptVec,
                                                           const unsigned int& col,
                                                           const unsigned int& row){

    PclPoint out_pt;
    float sum = 0.f; float x = 0.f; float y = 0.f;
    float z = 0.f; float r = 0.f; float g = 0.f; float b = 0.f;
    Vector2d nom_pt(x_coord + col * _square_size, y_coord + row * _square_size);
    float radius = 0.01;
    float std_dev = 0.04;
    float den = 2.f * std_dev * std_dev;
    int iter = 0;
    for( PclPoint pt : ptVec ){
        if(pt.xyz_.norm() > 1e-4){
            //float weight = ( nom_pt - pt.getVector3fMap().head(2) ).norm()/radius;
            //float weight =  float( exp ( - (double) ( ( nom_pt - pt.getVector3fMap().head(2) ).norm() ) / ( (double)(2.0 * radius * radius) ) ) );
            float dist = ( nom_pt - pt.xyz_.head(2) ).norm();
            float weight = 1 / exp ( dist / den );
            //setprecision(10);
            //std::cerr << setprecision(10) << nom_pt.transpose() << " " << pt.getVector3fMap().head(2).transpose() << " " << dist << " " << 1 / exp ( dist / (2.0 * 0.04 * 0.04) )  << "\n";
            sum += weight;
            x += pt.xyz_(0)*weight;
            y += pt.xyz_(1)*weight;
            z += pt.xyz_(2)*weight;
            r += (float)pt.color_(0)*weight;
            g += (float)pt.color_(1)*weight;
            b += (float)pt.color_(2)*weight;
            iter++;
        }
    }
    if(iter){
        out_pt.xyz_(0) = x/sum;
        out_pt.xyz_(1) = y/sum;
        out_pt.xyz_(2) = z/sum;
        out_pt.color_(0) = r/sum;
        out_pt.color_(1) = g/sum;
        out_pt.color_(2) = b/sum;
        return out_pt;
    }
}

void EnvironmentRepresentation::computeMMGridMap(){

    // Image size is flipped for the sake of better visualization of the results
    exgImg = cv::Mat( cv::Size(_height, _width), CV_8UC1, cv::Scalar(0) );
    elevImg = cv::Mat( cv::Size(_height, _width), CV_8UC1, cv::Scalar(0) );
    xyzImg = cv::Mat( cv::Size(_height, _width), CV_32FC3, cv::Scalar(0,0,0) );
    exgImgColor = cv::Mat( cv::Size(_height, _width), CV_8UC3, cv::Scalar(0,0,0) );
    rgbImg = cv::Mat( cv::Size(_height, _width), CV_8UC3, cv::Scalar(0,0,0) );

    int iter = 0;
    for(unsigned int r = 0; r < _height; ++r){
        for(unsigned int c = 0; c < _width; ++c, ++iter){
            PclPoint pt = computeAveragePoint(_gridMap[iter], c, r);
            if( pt.xyz_.norm() <= 1e-3 )
                continue;
            int ExG = computeExGforXYZRGBPoint(pt.color_);
            exgImg.at<uchar>(c,r) = ExG;
            exgImgColor.at<cv::Vec3b>(c,r)[1] = ExG;
            elevImg.at<uchar>(c,r) = altitude_scale * ( pt.xyz_(2) - minPt(2) );
            xyzImg.at<cv::Vec3f>(c,r)[0] = pt.xyz_(0);
            xyzImg.at<cv::Vec3f>(c,r)[1] = pt.xyz_(1);
            xyzImg.at<cv::Vec3f>(c,r)[2] = pt.xyz_(2);
            rgbImg.at<cv::Vec3b>(c,r)[0] = pt.color_(0)*255.f;
            rgbImg.at<cv::Vec3b>(c,r)[1] = pt.color_(1)*255.f;
            rgbImg.at<cv::Vec3b>(c,r)[2] = pt.color_(2)*255.f;
        }
    }

    cv::normalize(xyzImg, xyzImgUChar, 255, 0, cv::NORM_MINMAX);
    xyzImgUChar.convertTo(xyzImgUChar, CV_8UC3);

    /*cv::imshow("rgbImg_", rgbImg);
    cv::imshow("exgImg_", exgImg);
    cv::imshow("elevImg_", elevImg);
    cv::waitKey(0);
    cv::destroyAllWindows();*/
}

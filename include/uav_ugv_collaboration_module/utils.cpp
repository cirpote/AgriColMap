#include "utils.h"

std::vector<double> vectorFromString(const std::string& str) {

    std::vector<double> output;
    std::string offset_cpy = str;
    std::string delimiter = " ";
    size_t pos = 0;
    std::string token;
    while ((pos = offset_cpy.find(delimiter)) != std::string::npos) {
        token = offset_cpy.substr(0, pos);
        output.push_back( atof(token.c_str()) );
        offset_cpy.erase(0, pos + delimiter.length());
    }
    output.push_back( atof(offset_cpy.c_str()) );
    return output;
}



void AffineTransformFromString(const std::string& str,
                                      Eigen::Matrix3f& R,
                                      Eigen::Vector3f& t,
                                      Eigen::Vector2f& scale) {

    std::vector<float> output;
    std::string offset_cpy = str;
    std::string delimiter = " ";
    size_t pos = 0;
    std::string token;
    while ((pos = offset_cpy.find(delimiter)) != std::string::npos) {
        token = offset_cpy.substr(0, pos);
        output.push_back( atof(token.c_str()) );
        offset_cpy.erase(0, pos + delimiter.length());
    }
    output.push_back( atof(offset_cpy.c_str()) );

    R(0,0) = output[0]; R(0,1) = output[1]; R(0,2) = output[2]; t(0) = output[3];
    R(1,0) = output[4]; R(1,1) = output[5]; R(1,2) = output[6]; t(1) = output[7];
    R(2,0) = output[8]; R(2,1) = output[9]; R(2,2) = output[10]; t(2) = output[11];
    scale(0) = output[12]; scale(1) = output[13];
}


Vector3 getScaleFromAffineMatrix(const Matrix3& aff){

    Vector3 output;
    output(0) = std::sqrt( aff(0,0)*aff(0,0) + aff(1,0)*aff(1,0) + aff(2,0)*aff(2,0) );
    output(1) = std::sqrt( aff(1,0)*aff(1,0) + aff(1,1)*aff(1,1) + aff(2,1)*aff(2,1) );
    output(2) = std::sqrt( aff(2,0)*aff(2,0) + aff(1,2)*aff(1,2) + aff(2,2)*aff(2,2) );
    return output;
}

unsigned char computeExGforPoint(const PCLptXYZRGB& pt,
                                 PCLptXYZRGB_Vector& PointCloudFiltered,
                                 const Vector3i& color){

        float red, blue, green, sum, ExG, sumNorm = 0.f;
        sum = (int)pt.r  + (int)pt.g  + (int)pt.b;
        red = (int)pt.r / sum;
        green = (int)pt.g / sum;
        blue = (int)pt.b / sum;
        sumNorm = blue + green + red;
        ExG = (2 * green - blue - red) * 255.0;
        ExG = std::max(0.f, ExG);

        PCLptXYZRGB curr_point;
        curr_point.getVector3fMap() = pt.getVector3fMap();
        if(ExG > 30 ){
            curr_point.r = color(0);
            curr_point.g = color(1);
            curr_point.b = color(2);
            PointCloudFiltered.push_back( curr_point );
        }

        return ExG;
}

unsigned char computeExGforXYZRGBPoint(const PCLptXYZRGB& pt){

        float red, blue, green, sum, ExG, sumNorm = 0.f;
        sum = (int)pt.r  + (int)pt.g  + (int)pt.b;
        red = (int)pt.r / sum;
        green = (int)pt.g / sum;
        blue = (int)pt.b / sum;
        sumNorm = blue + green + red;
        ExG = (2 * green - blue - red) * 255.0;
        ExG = std::max(0.f, ExG);

        return ExG;
}

PCLptXYZRGB getHighestPoint(std::vector<int>& pt_list,
                            PCLptXYZRGB_Vector& pt_cloud){

    float max_z = -1000;
    int max_z_index = 0.f;
    for( int iter : pt_list){

        if (pt_cloud[iter].z > max_z){
            max_z = pt_cloud[iter].z;
            max_z_index = iter;
        }
    }

    return pt_cloud[max_z_index];
}

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
                                      Eigen::Matrix3d& R,
                                      Eigen::Vector3d& t,
                                      Eigen::Vector2d& scale) {

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

unsigned char computeExGforXYZRGBPoint(const Vector3d& pt){

        float red, blue, green, sum, ExG, sumNorm = 0.f;
        sum = (int)(pt(0) * 255)  + (int)(pt(1) * 255)  + (int)(pt(2) * 255);
        red = (int)(pt(0) * 255 ) / sum;
        green = (int)(pt(1) * 255 ) / sum;
        blue = (int)(pt(2) * 255 )/ sum;
        sumNorm = blue + green + red;
        ExG = (2 * green - blue - red) * 255.0;
        ExG = std::max(0.f, ExG);

        return ExG;
}

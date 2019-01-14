#pragma once
#include "defs.h"

Vector3 inline toVector3(const std::vector<float>& vector){

    return Vector3( vector[0], vector[1], vector[2]);
}

float inline computeAngle(const Matrix3& R) {
    float rotation_trace = R.trace();
    return acos(std::min(float(1), std::max(float(-1), (rotation_trace - 1)/2)));
}

void inline ExitWithErrorMsg(const std::string& msg){
    std::cerr << FRED( "ERROR: " ) << msg << "\n";
    std::exit(1);
}

inline float yawFromQuaternion(const Eigen::Quaternionf& q) {
   return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline float rollFromQuaternion(const Eigen::Quaternionf& q) {
   return atan2(2.0*(q.x()*q.w() + q.y()*q.z()),
               1 - 2*(q.x()*q.x() + q.y()*q.y()));
}

inline float pitchFromQuaternion(const Eigen::Quaternionf& q) {
   return asin(2.0*(q.w()*q.y() - q.x()*q.z()));
}


std::vector<double> vectorFromString(const std::string& str);

void AffineTransformFromString(const std::string& str,
                               Eigen::Matrix3d& R,
                               Eigen::Vector3d& t,
                               Eigen::Vector2d& scale);

Vector3 getScaleFromAffineMatrix(const Matrix3& aff);

unsigned char computeExGforXYZRGBPoint(const Vector3d& pt);
